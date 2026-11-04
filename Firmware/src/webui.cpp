#include "webui.h"
#include "main.h"
#include "settings.h"
#include "climate.h"
#include "homeassistant.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <web_files.h>

// Pages live in web/ and are compiled into the firmware by
// scripts/embed_web.py. They use {{name}} placeholders, which are
// HTML-escaped unless marked raw.

static ESP8266WebServer server(80);
static DNSServer dnsServer;
static bool inSetupMode = false;
static String hotspot;
static String networkButtons;

struct TemplateVar
{
  const char *key;
  String value;
  bool raw;
};

//###########################################
//  Helpers
//###########################################
static String HtmlEscape(const String &in)
{
  String out;
  out.reserve(in.length() + 8);
  for (unsigned int i = 0; i < in.length(); i++)
  {
    char c = in[i];
    switch (c)
    {
    case '&': out += F("&amp;"); break;
    case '<': out += F("&lt;"); break;
    case '>': out += F("&gt;"); break;
    case '"': out += F("&quot;"); break;
    case '\'': out += F("&#39;"); break;
    default: out += c;
    }
  }
  return out;
}

static const WebFile *FindWebFile(const char *path)
{
  for (const WebFile &file : WEB_FILES)
    if (strcmp(file.path, path) == 0)
      return &file;
  return nullptr;
}

static bool ReadPage(const char *path, String &page)
{
  const WebFile *file = FindWebFile(path);
  if (!file)
  {
    server.send(500, "text/plain", "Page missing from firmware");
    return false;
  }
  page = FPSTR(file->data);
  return true;
}

static void HandleStyle()
{
  const WebFile *file = FindWebFile("/style.css");
  if (!file)
  {
    server.send(404, "text/plain", "Not found");
    return;
  }
  // The pages link it with a content hash, so it can be cached for a long time.
  server.sendHeader("Cache-Control", "max-age=86400");
  server.send_P(200, file->type, file->data, file->length);
}

static void SendHtml(const String &page, int code)
{
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "text/html; charset=utf-8", page);
}

static void SendPage(const char *path, std::initializer_list<TemplateVar> vars, int code = 200)
{
  String page;
  if (!ReadPage(path, page))
    return;
  for (const TemplateVar &v : vars)
    page.replace(String("{{") + v.key + "}}", v.raw ? v.value : HtmlEscape(v.value));
  SendHtml(page, code);
}

// Same, with every value in a JSON object (all escaped).
static void SendPage(const char *path, JsonDocument &vars, int code = 200)
{
  String page;
  if (!ReadPage(path, page))
    return;
  for (JsonPair kv : vars.as<JsonObject>())
    page.replace(String("{{") + kv.key().c_str() + "}}", HtmlEscape(kv.value().as<String>()));
  SendHtml(page, code);
}

// refreshSeconds > 0 sends the browser to `next` after that long.
static void SendMessage(const String &title, const String &message, const char *next = "/",
                        int refreshSeconds = 0, int code = 200)
{
  String refresh;
  if (refreshSeconds > 0)
    refresh = "<meta http-equiv=\"refresh\" content=\"" + String(refreshSeconds) + ";url=" + next + "\">";
  String actions;
  if (next != nullptr)
    actions = "<a class=\"btn ghost\" href=\"" + String(next) + "\">Back</a>";

  SendPage("/message.html", {{"title", title, false}, {"message", message, false}, {"refresh", refresh, true}, {"actions", actions, true}}, code);
}

static String FormatReading(float value, const char *unit)
{
  if (!lastClimateValid || isnan(value))
    return "—";
  return String(value, 1) + unit;
}

static String FormatUptime()
{
  unsigned long s = UptimeSeconds();
  unsigned long d = s / 86400, h = (s / 3600) % 24, m = (s / 60) % 60;
  if (d > 0)
    return String(d) + "d " + h + "h";
  if (h > 0)
    return String(h) + "h " + m + "m";
  return String(m) + "m";
}

static String SignalBars(int rssi)
{
  int bars = rssi >= -55 ? 4 : rssi >= -67 ? 3 : rssi >= -78 ? 2 : 1;
  String out;
  for (int i = 0; i < 4; i++)
    out += i < bars ? "●" : "○"; // filled / empty circles
  return out;
}

static long ArgNumber(const char *name, long minValue, long maxValue)
{
  long value = server.arg(name).toInt();
  return value < minValue ? minValue : value > maxValue ? maxValue : value;
}

//###########################################
//  Form handling shared by setup and normal mode
//###########################################

// Returns an error message, or empty on success.
static String ReadWiFiForm()
{
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  ssid.trim();

  if (ssid.isEmpty())
    return "Enter the name of your Wi-Fi network.";
  if (password.length() > 0 && password.length() < 8)
    return "Wi-Fi passwords are at least 8 characters.";

  // A blank password keeps the saved one, unless the network changed.
  bool networkChanged = ssid != settings.wifiSsid;
  if (networkChanged || password.length() > 0)
  {
    settings.wifiPassword = password;
    settings.wifiVerified = false;
  }
  settings.wifiSsid = ssid;
  return "";
}

static void ReadMQTTForm()
{
  // Called from setup mode too, where there's no broker connection to clean up.
  settings.mqttHost = server.arg("mqttHost");
  settings.mqttPort = ArgNumber("mqttPort", 1, 65535);
  settings.mqttUser = server.arg("mqttUser");
  // Passwords are never sent back to the browser; blank keeps the saved one.
  if (server.arg("mqttPassword").length() > 0)
    settings.mqttPassword = server.arg("mqttPassword");
  settings.mqttBaseTopic = server.arg("baseTopic");
  if (server.hasArg("haPrefix"))
    settings.haDiscoveryPrefix = server.arg("haPrefix");
  NormalizeSettings();
}

static const char *KeepPasswordHint(const String &saved)
{
  return saved.isEmpty() ? "" : "Leave blank to keep the saved password";
}

//###########################################
//  Setup mode
//###########################################
void WebUIScanNetworks()
{
  Log.noticeln("Scanning for Wi-Fi networks");
  int count = WiFi.scanNetworks();
  networkButtons = "";

  // Strongest first, skipping hidden networks and duplicates (mesh nodes).
  std::vector<int> order;
  for (int i = 0; i < count; i++)
    order.push_back(i);
  std::sort(order.begin(), order.end(), [](int a, int b) { return WiFi.RSSI(a) > WiFi.RSSI(b); });

  std::vector<String> seen;
  for (int i : order)
  {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty() || std::find(seen.begin(), seen.end(), ssid) != seen.end())
      continue;
    seen.push_back(ssid);

    String name = HtmlEscape(ssid);
    String lock = WiFi.encryptionType(i) == ENC_TYPE_NONE ? "" : " \U0001F512";
    networkButtons += "<button type=\"button\" class=\"net\" data-ssid=\"" + name + "\">" + name +
                      "<span>" + lock + " " + SignalBars(WiFi.RSSI(i)) + "</span></button>";
    if (seen.size() >= 12)
      break;
  }
  WiFi.scanDelete();

  if (networkButtons.isEmpty())
    networkButtons = "<p class=\"hint\">No networks found. You can still type the name below.</p>";
}

static void HandleSetupPage()
{
  if (server.hasArg("scan"))
    WebUIScanNetworks();

  SendPage("/setup.html", {
                              {"hotspot", hotspot, false},
                              {"networks", networkButtons, true},
                              {"ssid", settings.wifiSsid, false},
                              {"wifiPasswordHint", KeepPasswordHint(settings.wifiPassword), false},
                              {"mqttHost", settings.mqttHost, false},
                              {"mqttPort", String(settings.mqttPort), false},
                              {"mqttUser", settings.mqttUser, false},
                              {"mqttPasswordHint", KeepPasswordHint(settings.mqttPassword), false},
                              {"baseTopic", settings.mqttBaseTopic, false},
                              {"haPrefix", settings.haDiscoveryPrefix, false},
                          });
}

static void HandleSetupSave()
{
  String error = ReadWiFiForm();
  if (!error.isEmpty())
  {
    SendMessage("Check your settings", error, "/", 0, 400);
    return;
  }
  ReadMQTTForm();

  if (!SaveSettings())
  {
    SendMessage("Couldn't save", "Saving the settings failed. Please try again.", "/", 0, 500);
    return;
  }

  SendMessage("All set",
              "The opener is restarting and joining \"" + settings.wifiSsid +
                  "\". Reconnect this phone or computer to your usual network, then open "
                  "http://garagedooropener.local/ or look for the device in Home Assistant. "
                  "If it can't connect, the " + hotspot + " hotspot will come back so you can try again.",
              nullptr);
  RequestRestart(2000);
}

// Captive portal: send every unknown URL (including phone connectivity
// checks) to the setup page so the sign-in prompt pops up.
static void HandleCaptiveRedirect()
{
  server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void WebUIBeginSetupMode(const String &hotspotName)
{
  inSetupMode = true;
  hotspot = hotspotName;

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/style.css", HTTP_GET, HandleStyle);
  server.on("/", HTTP_GET, HandleSetupPage);
  server.on("/save", HTTP_POST, HandleSetupSave);
  server.onNotFound(HandleCaptiveRedirect);
  server.begin();
}

//###########################################
//  Normal mode
//###########################################
// Values on the dashboard that change. The page renders them once and then
// refreshes them from /status.json; keys match the element ids in index.html.
static void FillLiveStatus(JsonDocument &doc)
{
  DeviceStatus status = GetDeviceStatus();

  doc["door"] = status.door;
  doc["doorClass"] = strcmp(status.door, "Closed") == 0 ? "on" : strcmp(status.door, "Unknown") == 0 ? "off" : "alert";
  doc["mqtt"] = !status.mqttConfigured ? "MQTT not set up" : status.mqttConnected ? "Connected" : "MQTT offline";
  doc["mqttClass"] = !status.mqttConfigured ? "off" : status.mqttConnected ? "on" : "alert";
  doc["motion"] = status.motion ? "Detected" : "Clear";
  doc["temperature"] = FormatReading(lastClimate.temperatureF, " °F");
  doc["humidity"] = FormatReading(lastClimate.humidity, "%");
  doc["heatIndex"] = FormatReading(lastClimate.heatIndexF, " °F");
  doc["bay1"] = !ParkingEnabled() ? "Off" : status.bay1 ? status.bay1 : "—";
  doc["bay2"] = !ParkingEnabled() ? "Off" : status.bay2 ? status.bay2 : "—";
  doc["signal"] = String(WiFi.RSSI()) + " dBm";
  doc["uptime"] = FormatUptime();
  doc["heap"] = String(ESP.getFreeHeap() / 1024) + " KB";
}

static void HandleStatus()
{
  JsonDocument vars;
  FillLiveStatus(vars);
  vars["ip"] = WiFi.localIP().toString();
  vars["network"] = WiFi.SSID();
  vars["version"] = FIRMWARE_VERSION;
  SendPage("/index.html", vars);
}

static void HandleStatusJson()
{
  JsonDocument doc;
  FillLiveStatus(doc);
  String body;
  serializeJson(doc, body);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", body);
}

static void HandleWiFiPage()
{
  SendPage("/wifi.html", {
                             {"ssid", settings.wifiSsid, false},
                             {"wifiPasswordHint", KeepPasswordHint(settings.wifiPassword), false},
                             {"hotspot", SetupHotspotName(), false},
                         });
}

static void HandleWiFiSave()
{
  String error = ReadWiFiForm();
  if (!error.isEmpty())
  {
    SendMessage("Check your settings", error, "/wifi", 0, 400);
    return;
  }
  if (!SaveSettings())
  {
    SendMessage("Couldn't save", "Saving the settings failed. Please try again.", "/wifi", 0, 500);
    return;
  }
  SendMessage("Restarting",
              "Joining \"" + settings.wifiSsid + "\". If it can't connect, the " + SetupHotspotName() +
                  " hotspot will start so you can fix the settings.",
              "/", 30);
  RequestRestart(2000);
}

static void HandleMQTTPage()
{
  SendPage("/mqtt.html", {
                             {"mqttHost", settings.mqttHost, false},
                             {"mqttPort", String(settings.mqttPort), false},
                             {"mqttUser", settings.mqttUser, false},
                             {"mqttPasswordHint", KeepPasswordHint(settings.mqttPassword), false},
                             {"baseTopic", settings.mqttBaseTopic, false},
                             {"haPrefix", settings.haDiscoveryPrefix, false},
                         });
}

static void HandleMQTTSave()
{
  const String oldHost = settings.mqttHost;
  const int oldPort = settings.mqttPort;
  const String oldBaseTopic = settings.mqttBaseTopic;
  const String oldPrefix = settings.haDiscoveryPrefix;

  ReadMQTTForm();

  // Clean up what's retained on the old broker/topics before switching, so
  // Home Assistant isn't left with a device that stays unavailable forever.
  bool leavingDiscovery = settings.mqttHost != oldHost || settings.mqttPort != oldPort ||
                          settings.haDiscoveryPrefix != oldPrefix;
  if (leavingDiscovery || settings.mqttBaseTopic != oldBaseTopic)
    ForgetMQTTTopics(oldBaseTopic, oldPrefix, leavingDiscovery);

  if (!SaveSettings())
  {
    SendMessage("Couldn't save", "Saving the settings failed. Please try again.", "/mqtt", 0, 500);
    return;
  }
  ApplyMQTTSettings();
  SendMessage("Saved", "MQTT settings saved. Reconnecting to the broker.", "/", 3);
}

static void HandleDevicePage()
{
  SendPage("/device.html", {
                               {"ping1WarnDist", String(settings.ping1WarnDist), false},
                               {"ping1StopDist", String(settings.ping1StopDist), false},
                               {"ping2WarnDist", String(settings.ping2WarnDist), false},
                               {"ping2StopDist", String(settings.ping2StopDist), false},
                               {"pingSpeedMS", String(settings.pingSpeedMS), false},
                               {"dhtSeconds", String(settings.dhtReadIntervalSeconds), false},
                           });
}

// Raw parking distances for the setup page. ?live=1 keeps fast readings on.
static void HandleParkingJson()
{
  if (server.hasArg("live"))
    StartParkingLiveReadings();

  DeviceStatus status = GetDeviceStatus();
  JsonDocument doc;
  doc["live"] = status.parkingLive;
  doc["door"] = status.door;
  if (status.bay1Cm >= 0)
    doc["bay1Cm"] = status.bay1Cm;
  if (status.bay2Cm >= 0)
    doc["bay2Cm"] = status.bay2Cm;
  doc["bay1"] = status.bay1 ? status.bay1 : "—";
  doc["bay2"] = status.bay2 ? status.bay2 : "—";

  String body;
  serializeJson(doc, body);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", body);
}

static void HandleDeviceSave()
{
  settings.ping1WarnDist = ArgNumber("ping1WarnDist", 0, 1000);
  settings.ping1StopDist = ArgNumber("ping1StopDist", 0, 1000);
  settings.ping2WarnDist = ArgNumber("ping2WarnDist", 0, 1000);
  settings.ping2StopDist = ArgNumber("ping2StopDist", 0, 1000);
  settings.pingSpeedMS = ArgNumber("pingSpeedMS", 0, 60000);
  settings.dhtReadIntervalSeconds = ArgNumber("dhtSeconds", 0, 86400);

  if (!SaveSettings())
  {
    SendMessage("Couldn't save", "Saving the settings failed. Please try again.", "/device", 0, 500);
    return;
  }
  ApplySensorSettings();
  SendMessage("Saved", "Sensor settings saved.", "/", 2);
}

static void HandleRestart()
{
  SendMessage("Restarting", "The opener will be back in a few seconds.", "/", 15);
  RequestRestart(1000);
}

static void HandleFactoryReset()
{
  ForgetMQTTTopics(settings.mqttBaseTopic, settings.haDiscoveryPrefix, true);
  EraseSettings();
  SendMessage("Settings erased",
              "The opener is restarting into setup mode. Connect to the " + SetupHotspotName() +
                  " Wi-Fi network to set it up again.",
              nullptr);
  RequestRestart(2000);
}

static void HandleNotFound()
{
  SendMessage("Not found", "That page doesn't exist.", "/", 0, 404);
}

void WebUIBeginNormalMode()
{
  inSetupMode = false;

  server.on("/style.css", HTTP_GET, HandleStyle);
  server.on("/", HTTP_GET, HandleStatus);
  server.on("/status.json", HTTP_GET, HandleStatusJson);
  server.on("/wifi", HTTP_GET, HandleWiFiPage);
  server.on("/wifi", HTTP_POST, HandleWiFiSave);
  server.on("/mqtt", HTTP_GET, HandleMQTTPage);
  server.on("/mqtt", HTTP_POST, HandleMQTTSave);
  server.on("/device", HTTP_GET, HandleDevicePage);
  server.on("/device", HTTP_POST, HandleDeviceSave);
  server.on("/parking.json", HTTP_GET, HandleParkingJson);
  server.on("/restart", HTTP_POST, HandleRestart);
  server.on("/reset", HTTP_POST, HandleFactoryReset);
  server.onNotFound(HandleNotFound);
  server.begin();
}

void WebUILoop()
{
  if (inSetupMode)
    dnsServer.processNextRequest();
  server.handleClient();
}
