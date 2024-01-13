#include "main.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NewPing.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <string.h>
#include <Ultrasonic.h>

//#########################################
//  WebServer Items
//#########################################

bool initialSetupComplete = false;
ESP8266WebServer server(80);

//##########################################
//  Pin Defs
//##########################################
const int LED_PIN = 2;
const int RELAY_PIN = 4;
const int MOTION_PIN = 13;
const int PING_1_TRIG_PIN = 16;
const int PING_2_TRIG_PIN = 3;
const int PING_ECHO_PIN = 12;
const int DHT_PIN = 1;
const int DOOR_OPEN_SENSOR_PIN = 5;
const int DOOR_CLOSED_SENSOR_PIN = 14;

//###########################################
//  DHT Defs
//###########################################
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
DHT dht(DHT_PIN, DHTTYPE);
int lastTemp = -99;
int lastHumidity = -99;
int lastHeatIndex = -99;
unsigned long lastSensorCheckMillis = 0;
bool sendTempUpdate = false;
bool sendHumidityUpdate = false;
bool sendHeatIndexUpdate = false;
unsigned long dhtReadIntervalMillis = 0;

//###########################################
//  WiFi Defs
//###########################################
WiFiClient espClient;
char *ssid;
char *password;

//###########################################
//  MQTT Defs
//###########################################
PubSubClient client(espClient);
bool isMQTTSetup = false;
String mqttUser;
String mqttPassword;
String mqttServer;
String mqttBaseTopic;
int mqttPort;

//###########################################
//  Ping Defs
//###########################################
#define DIST_STATE_UNKNOWN 0
#define DIST_STATE_NORMAL 1
#define DIST_STATE_WARN 2
#define DIST_STATE_STOP 3

Ultrasonic ping1(PING_1_TRIG_PIN, PING_ECHO_PIN);
Ultrasonic ping2(PING_2_TRIG_PIN, PING_ECHO_PIN);

unsigned long lastPingMillis = 0;
unsigned int lastPing1Dist = 0;
unsigned int lastPing2Dist = 0;
unsigned int lastPing1Status = DIST_STATE_UNKNOWN;
unsigned int lastPing2Status = DIST_STATE_UNKNOWN;

unsigned int ping1WarnDist = 0;
unsigned int ping1StopDist = 0;
unsigned int ping2WarnDist = 0;
unsigned int ping2StopDist = 0;
unsigned long pingSpeedMS = 0;

//###########################################
//  Door Defs
//###########################################
os_timer_t doorToggleTimer;
bool sendDoorStateUpdate = true;
bool toggleOpener = false;
bool initialDoorStatePublished = false;
volatile bool doorStateChanged = true;

enum DoorState
{
  Unknown,
  Open,
  Opening,
  Closed,
  Closing,
  Initial
};
volatile DoorState currentDoorState = Initial;
volatile DoorState lastDoorState = Initial;
unsigned long lastDoorStateChangeMillis = 0;

//###########################################
//  Motion Defs
//###########################################
volatile bool motionChangeDetected = true;
volatile bool currentMotionState = false;

//###########################################
//  Setup
//###########################################
void setup()
{
  delay(100);

  SetupPins();

  // if(!digitalRead(DOOR_CLOSED_SENSOR_PIN) && !digitalRead(DOOR_OPEN_SENSOR_PIN))
  // {
  //   APModeSetup();
  // }
  // else
  // {
  StationModeSetup();
  // }
}

void APModeSetup()
{
  // delay(1000);
  // WiFi.softAP(apssid);
  // WiFi.softAPConfig(IPAddress(192,168,101,1), IPAddress(0,0,0,0), IPAddress(255,255,255,0));

  // server.on("/", handleRoot);
  // server.on("/post", handleSubmit);
  // server.begin();

  // APLoop();
}

void APLoop()
{
  while (1)
  {
    server.handleClient();
  }
}

void StationModeSetup()
{
  LittleFS.begin(); // Start the SPI Flash Files System

  if (LittleFS.exists("/creds.txt"))
  {
    initialSetupComplete = true;
    WiFi.mode(WIFI_STA);
    WiFi.hostname("GarageDoorOpener");
    WiFi.setAutoReconnect(true);
    File file = LittleFS.open("creds.txt", "r");
    file.readStringUntil(',').toCharArray(ssid, 50);
    file.readString().toCharArray(password, 100);
    file.close();

    WiFi.begin(ssid, password);
  }
  else
  {
    delay(5000);
    ESP.restart();
  }

  if (LittleFS.exists("/pingConfig.txt"))
  {
    File file = LittleFS.open("pingConfig.txt", "r");
    ping1WarnDist = file.readStringUntil(',').toInt();
    ping1StopDist = file.readStringUntil(',').toInt();
    ping2WarnDist = file.readStringUntil(',').toInt();
    ping2StopDist = file.readStringUntil(',').toInt();
    pingSpeedMS = file.readStringUntil(',').toInt();
    file.close();
  }

  if (LittleFS.exists("/dhtConfig.txt"))
  {
    File file = LittleFS.open("dhtConfig.txt", "r");
    dhtReadIntervalMillis = file.readStringUntil(',').toInt();
    file.close();
  }

  int i = 0;
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    // Wait for the Wi-Fi to connect
    delay(250);
    if (++i > 20)
    {
      ESP.restart();
    }
  }

  MDNS.begin("GarageDoorOpener");

  server.on("/", handleRoot);
  server.on("/wifiSetup", handleWiFiSetup);
  server.on("/wifiSetupComplete", handleWiFiSetupComplete);
  server.on("/mqttConfig", handleMQTTConfig);
  server.on("/mqttConfigComplete", handleMQTTConfigComplete);
  server.on("/deviceConfig", handleDeviceConfig);
  server.on("/deviceConfigComplete", handleDeviceConfigComplete);

  // server.onNotFound([]() {                              // If the client requests any URI
  //   if (!handleFileRead(server.uri()))                  // send it if it exists
  //     server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  // });

  server.begin(); // Actually start the server

  SetupOTA();

  SetupMQTT();

  SetupDHT();
  // MQTTReconnect();
  SetupInterrupts();
  ReadDoorState();
  digitalWrite(PING_1_TRIG_PIN, LOW);
}

void loop()
{
  ArduinoOTA.handle();
  server.handleClient();
  if (!client.connected())
  {
    MQTTReconnect();
  }
  else
  {
    client.loop();
  }
  // if (!initialDoorStatePublished)
  // {
  //   publishMQTT("door", lastDoorState);
  //   initialDoorStatePublished = true;
  // }
  HandleDHT();
  HandlePing();
  HandleMotion();
  HandleDoorState();
  // PublishDoorState();
}

void SetupPins()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PING_1_TRIG_PIN, OUTPUT);
  pinMode(PING_2_TRIG_PIN, OUTPUT);
  pinMode(PING_ECHO_PIN, INPUT);
  pinMode(DOOR_CLOSED_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_OPEN_SENSOR_PIN, INPUT_PULLUP);
  // pinMode(DHT_PIN, INPUT);
}

//################################################
//  Motion
//################################################
// ICACHE_RAM_ATTR void MotionDetected()
// {
//   detachInterrupt(digitalPinToInterrupt(MOTION_PIN));
//   motionChangeDetected = true;
//   currentMotionState = digitalRead(MOTION_PIN);
// }

void HandleMotion()
{
  int newMotionState = digitalRead(MOTION_PIN);
  if (newMotionState != currentMotionState)
  {
    if (newMotionState)
      publishMQTT("motion", "Motion Detected");
    else
      publishMQTT("motion", "Clear");

    currentMotionState = newMotionState;
    // attachInterrupt(digitalPinToInterrupt(MOTION_PIN), MotionDetected, CHANGE);
  }
}

//################################################
//  Door State
//################################################

void ReadDoorState()
{
  int closedSensor = digitalRead(DOOR_CLOSED_SENSOR_PIN);
  int openSensor = digitalRead(DOOR_OPEN_SENSOR_PIN);

  if (closedSensor && openSensor)
  {
    if (lastDoorState != Closing && lastDoorState != Opening)
    {
      if (lastDoorState == Closed)
        currentDoorState = Opening;
      else if (lastDoorState == Open)
        currentDoorState = Closing;
      else
        currentDoorState = Unknown;
    }
  }
  else if (!closedSensor)
  {
    currentDoorState = Closed;
  }
  else if (!openSensor)
  {
    currentDoorState = Open;
  }
}

void HandleDoorState()
{
  //Startup || millis() rollover || new state within debounce time
  if (lastDoorStateChangeMillis == 0 || lastDoorStateChangeMillis > millis() || millis() - lastDoorStateChangeMillis > 500)
  {
    ReadDoorState();

    if (currentDoorState != lastDoorState)
    {
      switch (currentDoorState)
      {
      case Open:
        publishMQTT("door", "Open");
        break;
      case Closed:
        publishMQTT("door", "Closed");
        break;
      case Opening:
        publishMQTT("door", "Opening");
        break;
      case Closing:
        publishMQTT("door", "Closing");
        break;
      default:
        publishMQTT("door", "Unknown");
        break;
      }

      lastDoorState = currentDoorState;
      lastDoorStateChangeMillis = millis();
    }
  }
}

void HandleDoorToggle()
{
  digitalWrite(RELAY_PIN, HIGH);
  os_timer_arm(&doorToggleTimer, 500, false);
}

void HandleDoorOpen()
{
  if(currentDoorState == Closed || currentDoorState == Closed)
  {
    HandleDoorToggle();
  }
}

void HandleDoorClose()
{
  if(currentDoorState == Open || currentDoorState == Opening)
  {
    HandleDoorToggle();
  }
}

void DoorToggleTimerCallback(void *pArg)
{
  os_timer_disarm(&doorToggleTimer);
  digitalWrite(RELAY_PIN, LOW);
}

//################################################
//  MQTT
//################################################
void SetupMQTT()
{
  if (LittleFS.exists("mqttConfig.txt"))
  {

    File f = LittleFS.open("mqttConfig.txt", "r");
    mqttServer = f.readStringUntil(',');
    mqttPort = f.readStringUntil(',').toInt();
    mqttUser = f.readStringUntil(',');
    mqttPassword = f.readStringUntil(',');
    mqttBaseTopic = f.readStringUntil(',');

    client.setServer(mqttServer.c_str(), mqttPort);

    client.setCallback(MQTTCallback);
    isMQTTSetup = true;
  }
}

void MQTTCallback(char *topic, byte *payload, unsigned int length)
{

  for (unsigned int i = 0; i < length; i++)
  {
  }

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == 'T')
  {
    HandleDoorToggle();
  }
  else if((char)payload[0] == 'O')
  {
    HandleDoorOpen();
  }
  else if((char)payload[0] == 'C')
  {
    HandleDoorClose();
  }
}

void MQTTReconnect()
{
  if (isMQTTSetup)
  {
    // Loop until we're reconnected
    if (!client.connected() && WiFi.isConnected())
    {
      // Create a random client ID
      String clientId = WiFi.hostname();
      clientId += String(random(0xffff), HEX);
      // Attempt to connect
      if (client.connect(clientId.c_str(), mqttBaseTopic.c_str(), 1, true, "OFFLINE"))
      {

        int i = 11 + mqttBaseTopic.length();
        char buffer[i + 1];
        strcpy(buffer, mqttBaseTopic.c_str());
        strcat(buffer, "doorcommand");
        // Once connected, publish an announcement...
        // ... and resubscribe
        client.subscribe(buffer);
        client.publish(mqttBaseTopic.c_str(), "ONLINE", true);
        digitalWrite(LED_PIN, LOW);
      }
      else
      {
        digitalWrite(LED_PIN, HIGH);
      }
    }
  }
}

void publishMQTT(const char *topic, const char *message)
{
  // try
  // {
  if (isMQTTSetup && client.connected())
  {
    int i = strlen(topic) + mqttBaseTopic.length();
    char buffer[i + 1];
    strcpy(buffer, mqttBaseTopic.c_str());
    strcat(buffer, topic);

    client.publish(buffer, message);
  }

  // }
  // catch(const std::exception& e)
  // {

  // }
}

//#############################################################
//  DHT
//#############################################################
void SetupDHT()
{
  dht.begin();
}

void HandleDHT()
{

  if (dhtReadIntervalMillis > 0 &&
      (lastSensorCheckMillis + dhtReadIntervalMillis < millis() || lastSensorCheckMillis == 0))
  {
    ReadDHT();
    lastSensorCheckMillis = millis();
  }
}

void ReadDHT()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  // float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // if (isnan(h) || isnan(f))
  // {
  //   publishMQTT("environment", "DHT Fail!");
  //   return;
  // }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  // float hic = dht.computeHeatIndex(t, h, false);

  char buffer[6];

  // if ((int)f != lastTemp)
  // {
  lastTemp = f;
  itoa(lastTemp, buffer, 10);
  publishMQTT("temp", buffer);
  // }
  // if ((int)h != lastHumidity)
  // {
  lastHumidity = h;
  itoa(lastHumidity, buffer, 10);
  publishMQTT("humidity", buffer);
  // }
  // if ((int)hif != lastHeatIndex)
  // {
  lastHeatIndex = hif;
  itoa(lastHeatIndex, buffer, 10);
  publishMQTT("heatindex", buffer);
  sendHeatIndexUpdate = true;
  // }
}

//########################################################
//  Ping
//########################################################
void HandlePing()
{
  if (currentDoorState == Open && pingSpeedMS > 0)
  {
    if (pingSpeedMS > 0 && millis() > lastPingMillis + pingSpeedMS)
    {
      lastPing1Dist = ping1.read();
      char buffer[10];
      sprintf(buffer, "%u", lastPing1Dist);

      lastPing2Dist = ping2.read();
      sprintf(buffer, "%u", lastPing2Dist);

      lastPingMillis = millis();

      if (lastPing1Dist <= ping1StopDist)
      {
        if (lastPing1Status != DIST_STATE_STOP)
        {
          publishMQTT("bay1/parkingstate", "STOP");
          lastPing1Status = DIST_STATE_STOP;
        }
      }
      else if (lastPing1Dist <= ping1WarnDist)
      {
        if (lastPing1Status != DIST_STATE_WARN)
        {
          publishMQTT("bay1/parkingstate", "WARN");
          lastPing1Status = DIST_STATE_WARN;
        }
      }
      else
      {
        if (lastPing1Status != DIST_STATE_NORMAL)
        {
          publishMQTT("bay1/parkingstate", "NORMAL");
          lastPing1Status = DIST_STATE_NORMAL;
        }
      }

      if (lastPing2Dist <= ping2StopDist)
      {
        if (lastPing2Status != DIST_STATE_STOP)
        {
          publishMQTT("bay2/parkingstate", "STOP");
          lastPing2Status = DIST_STATE_STOP;
        }
      }
      else if (lastPing2Dist <= ping2WarnDist)
      {
        if (lastPing2Status != DIST_STATE_WARN)
        {
          publishMQTT("bay2/parkingstate", "WARN");
          lastPing2Status = DIST_STATE_WARN;
        }
      }
      else
      {
        if (lastPing2Status != DIST_STATE_NORMAL)
        {
          publishMQTT("bay2/parkingstate", "NORMAL");
          lastPing2Status = DIST_STATE_NORMAL;
        }
      }
    }
  }
}

//########################################################
//  WiFi
//########################################################
void SetupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.hostname("GarageController");
  WiFi.setAutoReconnect(true);

  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    ESP.restart();
  }
}

void SetupOTA()
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    {
      type = "filesystem";
    }
  });

  ArduinoOTA.onEnd([]() {
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });

  ArduinoOTA.onError([](ota_error_t error) {
  });

  ArduinoOTA.begin();
}

void SetupInterrupts()
{
  os_timer_setfn(&doorToggleTimer, DoorToggleTimerCallback, NULL);
}

//#######################################################################
//  WebServer Routines
//#######################################################################
String getContentType(String filename)
{ // convert the file extension to the MIME type
  if (filename.endsWith(".html"))
    return "text/html";
  else if (filename.endsWith(".css"))
    return "text/css";
  else if (filename.endsWith(".js"))
    return "application/javascript";
  else if (filename.endsWith(".ico"))
    return "image/x-icon";
  return "text/plain";
}

bool handleFileRead(String path, const std::map<String, String> &replacementWords)
{ // send the right file to the client (if it exists)
  if (path.endsWith("/"))
    path += "index.html";                    // If a folder is requested, send the index file
  String contentType = getContentType(path); // Get the MIME type
  if (LittleFS.exists(path))
  {                                       // If the file exists
    File file = LittleFS.open(path, "r"); // Open it
    String pageContents = file.readString();
    for (auto item : replacementWords)
    {
      pageContents.replace("[" + item.first + "]", item.second);
    }
    server.send(200, getContentType(path), pageContents); // And send it to the client
    file.close();                                         // Then close the file again
    return true;
  }
  return false; // If the file doesn't exist, return false
}

bool handleFileRead(String path)
{
  return handleFileRead(path, std::map<String, String>());
}

void handleRoot()
{
  if (initialSetupComplete)
  {
    String path = "index.html";
    std::map<String, String> m =
        {
            {"lastTemp", String(lastTemp)},
            {"lastHumidity", String(lastHumidity)},
            {"lastHeatIndex", String(lastHeatIndex)}};
    handleFileRead(path, m);
  }
  else
  {
    server.sendHeader("Location", "wifiSetup", true);
    server.send(302, "text/plain", "");
  }
}

void handleWiFiSetup()
{
  String path = "wifiSetup.html";

  std::map<String, String> m = {
      {"ssid", WiFi.SSID()}};
  handleFileRead(path, m);
}

void handleWiFiSetupComplete()
{

  String newssid = server.arg("ssid");
  String newpassword = server.arg("password");

  File file = LittleFS.open("creds.txt", "w+");
  file.print(newssid);
  file.print(',');
  file.print(newpassword);
  file.close();

  String path = "wifiSetupComplete.html";
  handleFileRead(path);
  delay(5000);
  ESP.restart();
}

void handleMQTTConfig()
{
  String path = "mqttConfig.html";

  std::map<String, String> m = {
      {"server", mqttServer},
      {"port", String(mqttPort)},
      {"username", mqttUser},
      {"baseTopic", mqttBaseTopic}};

  handleFileRead(path, m);
}

void handleMQTTConfigComplete()
{
  mqttServer = server.arg("server");
  mqttPort = server.arg("port").toInt();
  mqttUser = server.arg("username");
  mqttPassword = server.arg("password");
  mqttBaseTopic = server.arg("baseTopic");

  if (!mqttBaseTopic.isEmpty() && mqttBaseTopic[mqttBaseTopic.length() - 1] != '/')
  {
    mqttBaseTopic = mqttBaseTopic + '/';
  }

  File file = LittleFS.open("mqttConfig.txt", "w+");
  file.print(mqttServer);
  file.print(',');
  file.print(mqttPort);
  file.print(',');
  file.print(mqttUser);
  file.print(',');
  file.print(mqttPassword);
  file.print(',');
  file.print(mqttBaseTopic);
  file.print(',');

  file.close();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleDeviceConfig()
{
  String path = "deviceConfig.html";

  std::map<String, String> m =
      {
          {"ping1WarnDist", String(ping1WarnDist)},
          {"ping1StopDist", String(ping1StopDist)},
          {"ping2WarnDist", String(ping2WarnDist)},
          {"ping2StopDist", String(ping2StopDist)},
          {"pingSpeedMS", String(pingSpeedMS)},
          {"dhtReadTimeSeconds", String(dhtReadIntervalMillis / 1000)}};

  handleFileRead(path, m);
}

void handleDeviceConfigComplete()
{
  ping1WarnDist = server.arg("ping1WarnDist").toInt();
  ping1StopDist = server.arg("ping1StopDist").toInt();
  ping2WarnDist = server.arg("ping2WarnDist").toInt();
  ping2StopDist = server.arg("ping2StopDist").toInt();
  pingSpeedMS = server.arg("pingSpeedMS").toInt();
  dhtReadIntervalMillis = server.arg("dhtReadTimeSeconds").toInt() * 1000;

  File file = LittleFS.open("pingConfig.txt", "w+");
  file.print(ping1WarnDist);
  file.print(',');
  file.print(ping1StopDist);
  file.print(',');
  file.print(ping2WarnDist);
  file.print(',');
  file.print(ping2StopDist);
  file.print(',');
  file.print(pingSpeedMS);
  file.print(',');

  file.close();

  file = LittleFS.open("dhtConfig.txt", "w+");
  file.print(dhtReadIntervalMillis);
  file.print(',');
  file.close();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
