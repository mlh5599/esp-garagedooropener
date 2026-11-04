#include "main.h"
#include <Arduino.h>
#include <pinDefinitions.h>
#include <climate.h>
#include <homeassistant.h>
#include <settings.h>
#include <webui.h>
#include <ArduinoLog.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Ultrasonic.h>

#define DEVICE_HOSTNAME "GarageDoorOpener"

// How long to wait for Wi-Fi at boot. New (unverified) credentials that
// don't connect in time drop back to setup mode; verified ones restart and retry.
#define WIFI_NEW_CREDENTIALS_TIMEOUT_MS 30000UL
#define WIFI_KNOWN_CREDENTIALS_TIMEOUT_MS 60000UL

// When setup mode was entered because new credentials failed, restart and
// try them again after this long (the router may just have been down).
#define SETUP_MODE_RETRY_MS (5UL * 60UL * 1000UL)

// MQTT reconnects back off from 5s to 60s while the broker is unreachable,
// since each attempt blocks the loop (DNS + TCP connect + CONNACK wait).
#define MQTT_RETRY_MIN_MS 5000UL
#define MQTT_RETRY_MAX_MS 60000UL
#define MQTT_CONNECT_TIMEOUT_MS 3000
#define MQTT_CONNACK_TIMEOUT_S 5

#define DIAGNOSTICS_INTERVAL_MS 60000UL

#define DOOR_DEBOUNCE_MS 500UL
// A door still between the sensors after this long has stopped part way.
#define DOOR_TRAVEL_TIMEOUT_MS 30000UL
// Live parking readings (setup page): one bay is read per tick, so each bay
// updates every 2 ticks. Live mode ends this long after the page's last poll.
#define PARKING_LIVE_TICK_MS 100UL
#define PARKING_LIVE_HOLD_MS 5000UL

//###########################################
//  Mode / restart
//###########################################
bool setupMode = false;
unsigned long setupModeStartMillis = 0;
unsigned long setupModeRetryMs = 0; // 0 = stay in setup mode until configured
bool restartRequested = false;
unsigned long restartRequestMillis = 0;
unsigned long restartDelayMs = 0;

//###########################################
//  MQTT Defs
//###########################################
WiFiClient espClient;
PubSubClient client(espClient);
bool isMQTTSetup = false;
bool haAnnouncePending = false;
unsigned long lastMQTTAttemptMillis = 0;
unsigned long mqttRetryDelayMs = MQTT_RETRY_MIN_MS;
unsigned long lastDiagnosticsMillis = 0;

//###########################################
//  Parking Defs
//###########################################
#define DIST_STATE_UNKNOWN 0
#define DIST_STATE_NORMAL 1
#define DIST_STATE_WARN 2
#define DIST_STATE_STOP 3

struct ParkingBay
{
  Ultrasonic sensor;
  const char *topic;
  unsigned int status;
  int distanceCm; // last raw reading: -1 = none yet, 0 = no echo
};

ParkingBay bay1 = {Ultrasonic(PING_1_TRIG_PIN, PING_ECHO_PIN), "bay1/parkingstate", DIST_STATE_UNKNOWN, -1};
ParkingBay bay2 = {Ultrasonic(PING_2_TRIG_PIN, PING_ECHO_PIN), "bay2/parkingstate", DIST_STATE_UNKNOWN, -1};
unsigned long lastPingMillis = 0;
bool parkingLive = false;
unsigned long parkingLiveMillis = 0;
bool liveReadBay2 = false;

//###########################################
//  Door Defs
//###########################################
os_timer_t doorToggleTimer;

enum DoorState
{
  Unknown,
  Open,
  Opening,
  Closed,
  Closing,
  Stopped, // between the sensors and no longer moving; reported as Open
  Initial
};
const char *DoorStateName(DoorState state);
const char *ParkingStateName(unsigned int status);

DoorState currentDoorState = Initial;
DoorState lastDoorState = Initial;
unsigned long lastDoorStateChangeMillis = 0;

//###########################################
//  Motion Defs
//###########################################
bool currentMotionState = false;

//###########################################
//  Setup
//###########################################
void setup()
{
  delay(100);

  // Serial is off by default: its TX/RX pins drive the DHT (GPIO1) and the
  // bay 2 sensor (GPIO3). Without Log.begin, Log calls stay silent.
  // See build_flags in platformio.ini.
#ifdef SERIAL_LOGGING
  Serial.begin(9600);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.noticeln("Logging Initialized");
#endif

  SetupPins();
  MountFileSystem();
  LoadSettings();

  // config.json is the only place credentials live; stop the Wi-Fi SDK from
  // saving its own copy to flash and auto-connecting with stale ones.
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);

  // Grounding both door sensor inputs at power-up (a state a real door can't
  // produce) forces setup mode, e.g. after the Wi-Fi password changes.
  delay(10);
  bool forceSetup = !digitalRead(DOOR_CLOSED_SENSOR_PIN) && !digitalRead(DOOR_OPEN_SENSOR_PIN);

  if (forceSetup)
  {
    Log.noticeln("Both door sensor inputs grounded; entering setup mode");
    StartSetupMode(0);
  }
  else if (!HasWiFiSettings())
  {
    Log.noticeln("No Wi-Fi settings; entering setup mode");
    StartSetupMode(0);
  }
  else
  {
    StartNormalMode();
  }
}

String SetupHotspotName()
{
  char name[24];
  snprintf(name, sizeof(name), "GarageDoor-%06X", ESP.getChipId());
  return String(name);
}

void StartSetupMode(unsigned long retryAfterMs)
{
  setupMode = true;
  setupModeStartMillis = millis();
  setupModeRetryMs = retryAfterMs;

  // The station side stays enabled only so the setup page can list networks.
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WebUIScanNetworks();

  String hotspot = SetupHotspotName();
  WiFi.softAP(hotspot.c_str());
  Log.noticeln("Setup hotspot %s started, open http://%s/", hotspot.c_str(), WiFi.softAPIP().toString().c_str());

  WebUIBeginSetupMode(hotspot);
}

bool WaitForWiFi(unsigned long timeoutMs)
{
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - start > timeoutMs)
      return false;
    delay(250);
  }
  return true;
}

void StartNormalMode()
{
  Log.noticeln("Connecting to Wi-Fi network %s", settings.wifiSsid.c_str());

  WiFi.mode(WIFI_STA);
  // Always powered, so trade a little power for snappier commands and pages.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str());

  if (!WaitForWiFi(settings.wifiVerified ? WIFI_KNOWN_CREDENTIALS_TIMEOUT_MS : WIFI_NEW_CREDENTIALS_TIMEOUT_MS))
  {
    if (!settings.wifiVerified)
    {
      Log.noticeln("Couldn't connect with the new Wi-Fi settings; entering setup mode");
      StartSetupMode(SETUP_MODE_RETRY_MS);
      return;
    }
    Log.noticeln("Couldn't connect to Wi-Fi; restarting to try again");
    delay(250);
    ESP.restart();
  }

  Log.noticeln("Connected, IP address: %s", WiFi.localIP().toString().c_str());

  if (!settings.wifiVerified)
  {
    settings.wifiVerified = true;
    SaveSettings();
  }

  MDNS.begin(DEVICE_HOSTNAME);
  WebUIBeginNormalMode();
  SetupOTA();
  MDNS.addService("http", "tcp", 80);

  SetupDHT();
  ApplySensorSettings();
  ApplyMQTTSettings();
  SetupInterrupts();
  ReadDoorState();
  digitalWrite(PING_1_TRIG_PIN, LOW);
}

void RequestRestart(unsigned long delayMs)
{
  // Restart from loop() so the web response has time to reach the browser.
  restartRequested = true;
  restartRequestMillis = millis();
  restartDelayMs = delayMs;
}

unsigned long UptimeSeconds()
{
  // 64-bit so it doesn't wrap after 49 days like millis().
  return (unsigned long)(micros64() / 1000000ULL);
}

//###########################################
//  Loop
//###########################################
void loop()
{
  WebUILoop();

  if (restartRequested && millis() - restartRequestMillis >= restartDelayMs)
  {
    Log.noticeln("Restarting");
    delay(100);
    ESP.restart();
  }

  if (setupMode)
  {
    // Slow blink while waiting to be set up (LED is active low)
    digitalWrite(LED_PIN, (millis() / 1000) % 2 ? HIGH : LOW);

    if (setupModeRetryMs > 0 && millis() - setupModeStartMillis > setupModeRetryMs && WiFi.softAPgetStationNum() == 0)
    {
      Log.noticeln("Setup mode timed out; restarting to retry Wi-Fi");
      ESP.restart();
    }
    return;
  }

  ArduinoOTA.handle(); // also services mDNS
  if (!client.connected())
  {
    MQTTReconnect();
  }
  else
  {
    client.loop();
  }

  if (haAnnouncePending && client.connected())
  {
    AnnounceToHomeAssistant();
  }
  if (client.connected() && millis() - lastDiagnosticsMillis >= DIAGNOSTICS_INTERVAL_MS)
  {
    PublishDiagnostics();
  }
  ClimateData climate;
  if (CheckDHT(climate))
  {
    PublishClimate(climate);
  }
  HandlePing();
  HandleMotion();
  HandleDoorState();
}

void SetupPins()
{
  Log.noticeln("Setting up pins");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PING_1_TRIG_PIN, OUTPUT);
  pinMode(PING_2_TRIG_PIN, OUTPUT);
  pinMode(PING_ECHO_PIN, INPUT);
  pinMode(DOOR_CLOSED_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_OPEN_SENSOR_PIN, INPUT_PULLUP);
  Log.noticeln("Pins setup");
}

void ApplySensorSettings()
{
  dhtReadIntervalMillis = settings.dhtReadIntervalSeconds * 1000UL;
  // The climate sensors' expiry in Home Assistant follows the read interval.
  if (client.connected())
    haAnnouncePending = true;
}

DeviceStatus GetDeviceStatus()
{
  DeviceStatus status;
  status.door = DoorStateName(currentDoorState);
  status.motion = currentMotionState;
  status.bay1 = ParkingStateName(bay1.status);
  status.bay2 = ParkingStateName(bay2.status);
  status.bay1Cm = bay1.distanceCm;
  status.bay2Cm = bay2.distanceCm;
  status.parkingLive = ParkingLiveActive();
  status.mqttConfigured = isMQTTSetup;
  status.mqttConnected = client.connected();
  return status;
}

//################################################
//  Motion
//################################################
void HandleMotion()
{
  bool newMotionState = digitalRead(MOTION_PIN);
  if (newMotionState != currentMotionState)
  {
    publishMQTT("motion", newMotionState ? "Motion Detected" : "Clear");
    currentMotionState = newMotionState;
  }
}

//################################################
//  Door State
//################################################
const char *DoorStateName(DoorState state)
{
  switch (state)
  {
  case Open:
  case Stopped: // Home Assistant treats a partly open door as open
    return "Open";
  case Closed:
    return "Closed";
  case Opening:
    return "Opening";
  case Closing:
    return "Closing";
  default:
    return "Unknown";
  }
}

void ReadDoorState()
{
  int closedSensor = digitalRead(DOOR_CLOSED_SENSOR_PIN);
  int openSensor = digitalRead(DOOR_OPEN_SENSOR_PIN);

  if (!closedSensor)
  {
    currentDoorState = Closed;
  }
  else if (!openSensor)
  {
    currentDoorState = Open;
  }
  else if (lastDoorState == Closed)
  {
    currentDoorState = Opening;
  }
  else if (lastDoorState == Open)
  {
    currentDoorState = Closing;
  }
  else if (lastDoorState != Opening && lastDoorState != Closing && lastDoorState != Stopped)
  {
    // Between the sensors with no history (e.g. at boot)
    currentDoorState = Unknown;
  }
}

void HandleDoorState()
{
  if (lastDoorStateChangeMillis != 0 && millis() - lastDoorStateChangeMillis < DOOR_DEBOUNCE_MS)
    return;

  ReadDoorState();

  if ((currentDoorState == Opening || currentDoorState == Closing) &&
      millis() - lastDoorStateChangeMillis > DOOR_TRAVEL_TIMEOUT_MS)
  {
    currentDoorState = Stopped;
  }

  if (currentDoorState != lastDoorState)
  {
    publishMQTT("door", DoorStateName(currentDoorState));

    lastDoorState = currentDoorState;
    lastDoorStateChangeMillis = millis();
  }
}

void HandleDoorToggle()
{
  digitalWrite(RELAY_PIN, HIGH);
  os_timer_arm(&doorToggleTimer, 500, false);
}

void HandleDoorOpen()
{
  if (currentDoorState == Closed || currentDoorState == Closing)
  {
    HandleDoorToggle();
  }
}

void HandleDoorClose()
{
  if (currentDoorState == Open || currentDoorState == Opening || currentDoorState == Stopped)
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
void ApplyMQTTSettings()
{
  client.disconnect();
  // PubSubClient keeps this pointer, so call this again whenever mqttHost changes.
  client.setServer(settings.mqttHost.c_str(), settings.mqttPort);
  client.setCallback(MQTTCallback);
  espClient.setTimeout(MQTT_CONNECT_TIMEOUT_MS); // DNS lookup and TCP connect
  client.setSocketTimeout(MQTT_CONNACK_TIMEOUT_S);
  isMQTTSetup = !settings.mqttHost.isEmpty();
  lastMQTTAttemptMillis = 0;
  mqttRetryDelayMs = MQTT_RETRY_MIN_MS;
  if (!isMQTTSetup)
    digitalWrite(LED_PIN, HIGH);
}

void MQTTCallback(char *topic, byte *payload, unsigned int length)
{
  if (length == 0)
    return;

  // Home Assistant publishes "online" here when it (re)starts. Re-announce
  // from loop() rather than here, since publishing reuses the receive buffer.
  if (HAStatusTopic(settings.haDiscoveryPrefix) == topic)
  {
    if (length == 6 && memcmp(payload, "online", 6) == 0)
      haAnnouncePending = true;
    return;
  }

  // Door commands: OPEN, CLOSE, TOGGLE (only the first character is checked)
  if ((char)payload[0] == 'T')
  {
    HandleDoorToggle();
  }
  else if ((char)payload[0] == 'O')
  {
    HandleDoorOpen();
  }
  else if ((char)payload[0] == 'C')
  {
    HandleDoorClose();
  }
}

void MQTTReconnect()
{
  if (!isMQTTSetup || client.connected() || !WiFi.isConnected())
    return;

  if (lastMQTTAttemptMillis != 0 && millis() - lastMQTTAttemptMillis < mqttRetryDelayMs)
    return;
  lastMQTTAttemptMillis = millis();

  // A stable client ID lets the broker replace a stale session from before a
  // power cut instead of keeping both.
  String clientId = HADeviceId();

  // Attempt to connect (with credentials if configured), using the base
  // topic as the last will so Home Assistant sees the device go offline.
  const char *user = settings.mqttUser.isEmpty() ? nullptr : settings.mqttUser.c_str();
  const char *pass = settings.mqttPassword.isEmpty() ? nullptr : settings.mqttPassword.c_str();
  const char *willTopic = settings.mqttBaseTopic.c_str();
  if (client.connect(clientId.c_str(), user, pass, willTopic, 1, true, "OFFLINE"))
  {
    mqttRetryDelayMs = MQTT_RETRY_MIN_MS;
    String doorCommandTopic = settings.mqttBaseTopic + "doorcommand";

    // Clear any retained command before subscribing. Otherwise the broker
    // would replay it on every reconnect and move the door.
    client.publish(doorCommandTopic.c_str(), "", true);
    client.subscribe(doorCommandTopic.c_str());
    client.subscribe(HAStatusTopic(settings.haDiscoveryPrefix).c_str());
    client.publish(willTopic, "ONLINE", true);
    AnnounceToHomeAssistant();
    digitalWrite(LED_PIN, LOW);
  }
  else
  {
    mqttRetryDelayMs = min(mqttRetryDelayMs * 2, MQTT_RETRY_MAX_MS);
    Log.noticeln("MQTT connection failed, state %d; retrying in %lus", client.state(), mqttRetryDelayMs / 1000);
    digitalWrite(LED_PIN, HIGH);
  }
}

void ForgetMQTTTopics(const String &baseTopic, const String &discoveryPrefix, bool removeDevice)
{
  // Stay off the broker until ApplyMQTTSettings() (or a restart) sets it up again.
  isMQTTSetup = false;
  if (!client.connected())
    return;
  if (removeDevice)
    ClearHADiscovery(client, discoveryPrefix);
  client.publish(baseTopic.c_str(), "", true); // retained ONLINE/OFFLINE
  // A clean disconnect stops the broker from publishing the retained
  // "OFFLINE" last will back onto the topic we just cleared.
  client.disconnect();
}

void publishMQTT(const char *topic, const char *message)
{
  if (isMQTTSetup && client.connected())
  {
    String fullTopic = settings.mqttBaseTopic + topic;
    client.publish(fullTopic.c_str(), message);
  }
}

void PublishClimate(const ClimateData &data)
{
  publishMQTT("temperature", String(data.temperatureF, 1).c_str());
  publishMQTT("humidity", String(data.humidity, 1).c_str());
  publishMQTT("heatindex", String(data.heatIndexF, 1).c_str());
}

void PublishDiagnostics()
{
  lastDiagnosticsMillis = millis();
  publishMQTT("rssi", String(WiFi.RSSI()).c_str());
  publishMQTT("uptime", String(UptimeSeconds()).c_str());
}

const char *ParkingStateName(unsigned int status)
{
  switch (status)
  {
  case DIST_STATE_NORMAL:
    return "NORMAL";
  case DIST_STATE_WARN:
    return "WARN";
  case DIST_STATE_STOP:
    return "STOP";
  default:
    return nullptr;
  }
}

// State messages aren't retained, so after (re)connecting or after Home
// Assistant restarts, send everything we currently know.
void PublishAllStates()
{
  publishMQTT("door", DoorStateName(currentDoorState));
  publishMQTT("motion", currentMotionState ? "Motion Detected" : "Clear");

  if (lastClimateValid)
    PublishClimate(lastClimate);

  if (ParkingEnabled())
  {
    for (ParkingBay *bay : {&bay1, &bay2})
    {
      const char *name = ParkingStateName(bay->status);
      publishMQTT(bay->topic, name ? name : "UNKNOWN");
    }
  }

  PublishDiagnostics();
}

void AnnounceToHomeAssistant()
{
  haAnnouncePending = false;
  PublishHADiscovery(client, settings.mqttBaseTopic, settings.haDiscoveryPrefix, settings.dhtReadIntervalSeconds,
                     ParkingEnabled());
  PublishAllStates();
}

//########################################################
//  Parking
//########################################################
void SetBayStatus(ParkingBay &bay, unsigned int status)
{
  if (bay.status == status)
    return;
  bay.status = status;
  if (!ParkingEnabled())
    return; // not in Home Assistant
  const char *name = ParkingStateName(status);
  publishMQTT(bay.topic, name ? name : "UNKNOWN");
}

bool ParkingEnabled()
{
  return settings.pingSpeedMS > 0;
}

void StartParkingLiveReadings()
{
  parkingLive = true;
  parkingLiveMillis = millis();
}

bool ParkingLiveActive()
{
  if (parkingLive && millis() - parkingLiveMillis >= PARKING_LIVE_HOLD_MS)
    parkingLive = false;
  return parkingLive;
}

// Takes a reading. When `report` is false only the raw distance is updated.
void ReadBay(ParkingBay &bay, unsigned int warnDist, unsigned int stopDist, bool report)
{
  // 0 means no echo came back: the sensor is missing or failed.
  unsigned int dist = bay.sensor.read();
  bay.distanceCm = (int)dist;
  if (!report)
    return;
  if (dist == 0)
    SetBayStatus(bay, DIST_STATE_UNKNOWN);
  else if (dist <= stopDist)
    SetBayStatus(bay, DIST_STATE_STOP);
  else if (dist <= warnDist)
    SetBayStatus(bay, DIST_STATE_WARN);
  else
    SetBayStatus(bay, DIST_STATE_NORMAL);
}

void HandlePing()
{
  // Parking states are only reported while the door is fully open.
  bool report = currentDoorState == Open && settings.pingSpeedMS > 0;
  if (!report)
  {
    SetBayStatus(bay1, DIST_STATE_UNKNOWN);
    SetBayStatus(bay2, DIST_STATE_UNKNOWN);
  }

  if (ParkingLiveActive())
  {
    // Fast readings for setting the distances, whatever the door is doing.
    // The bays share an echo line, so alternate to let each ping die out.
    if (millis() - lastPingMillis < PARKING_LIVE_TICK_MS)
      return;
    lastPingMillis = millis();
    if (liveReadBay2)
      ReadBay(bay2, settings.ping2WarnDist, settings.ping2StopDist, report);
    else
      ReadBay(bay1, settings.ping1WarnDist, settings.ping1StopDist, report);
    liveReadBay2 = !liveReadBay2;
    return;
  }

  if (!report || millis() - lastPingMillis < settings.pingSpeedMS)
    return;
  lastPingMillis = millis();

  ReadBay(bay1, settings.ping1WarnDist, settings.ping1StopDist, true);
  ReadBay(bay2, settings.ping2WarnDist, settings.ping2StopDist, true);
}

//########################################################
//  OTA / timers
//########################################################
void SetupOTA()
{
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA.onStart([]() {
    Log.noticeln("OTA update starting");
  });

  ArduinoOTA.onEnd([]() {
    Log.noticeln("OTA update finished");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Log.errorln("OTA update failed: %d", error);
  });

  ArduinoOTA.begin();
}

void SetupInterrupts()
{
  os_timer_setfn(&doorToggleTimer, DoorToggleTimerCallback, NULL);
}
