#include "settings.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <LittleFS.h>

static const char *CONFIG_PATH = "/config.json";
static const char *CONFIG_TEMP_PATH = "/config.tmp";

Settings settings;

bool MountFileSystem()
{
  if (LittleFS.begin())
    return true;
  Log.errorln("Failed to mount file system");
  return false;
}

bool HasWiFiSettings()
{
  return !settings.wifiSsid.isEmpty();
}

void NormalizeSettings()
{
  settings.mqttHost.trim();
  settings.mqttBaseTopic.trim();
  settings.haDiscoveryPrefix.trim();

  if (settings.mqttBaseTopic.isEmpty())
    settings.mqttBaseTopic = DEFAULT_MQTT_BASE_TOPIC;
  if (!settings.mqttBaseTopic.endsWith("/"))
    settings.mqttBaseTopic += '/';

  while (settings.haDiscoveryPrefix.endsWith("/"))
    settings.haDiscoveryPrefix.remove(settings.haDiscoveryPrefix.length() - 1);
  if (settings.haDiscoveryPrefix.isEmpty())
    settings.haDiscoveryPrefix = HA_DEFAULT_DISCOVERY_PREFIX;

  if (settings.mqttPort <= 0 || settings.mqttPort > 65535)
    settings.mqttPort = 1883;
}

bool LoadSettings()
{
  settings = Settings();

  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file)
  {
    Log.noticeln("No config file found");
    return false;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error)
  {
    Log.errorln("Failed to parse config file: %s", error.c_str());
    return false;
  }

  settings.wifiSsid = doc["wifi_ssid"] | "";
  settings.wifiPassword = doc["wifi_password"] | "";
  // Config files written before this flag existed were already working.
  settings.wifiVerified = doc["wifi_verified"] | true;

  settings.mqttHost = doc["mqtt_host"] | "";
  settings.mqttPort = doc["mqtt_port"] | 1883;
  settings.mqttUser = doc["mqtt_username"] | "";
  settings.mqttPassword = doc["mqtt_password"] | "";
  settings.mqttBaseTopic = doc["mqtt_base_topic"] | DEFAULT_MQTT_BASE_TOPIC;
  settings.haDiscoveryPrefix = doc["ha_discovery_prefix"] | HA_DEFAULT_DISCOVERY_PREFIX;

  settings.ping1WarnDist = doc["ping1_warn_dist"] | 0;
  settings.ping1StopDist = doc["ping1_stop_dist"] | 0;
  settings.ping2WarnDist = doc["ping2_warn_dist"] | 0;
  settings.ping2StopDist = doc["ping2_stop_dist"] | 0;
  settings.pingSpeedMS = doc["ping_speed_ms"] | 0UL;
  settings.dhtReadIntervalSeconds = doc["dht_read_interval_seconds"] | 0UL;

  NormalizeSettings();
  Log.noticeln("Config file read");
  return true;
}

bool SaveSettings()
{
  NormalizeSettings();

  JsonDocument doc;
  doc["wifi_ssid"] = settings.wifiSsid;
  doc["wifi_password"] = settings.wifiPassword;
  doc["wifi_verified"] = settings.wifiVerified;

  doc["mqtt_host"] = settings.mqttHost;
  doc["mqtt_port"] = settings.mqttPort;
  doc["mqtt_username"] = settings.mqttUser;
  doc["mqtt_password"] = settings.mqttPassword;
  doc["mqtt_base_topic"] = settings.mqttBaseTopic;
  doc["ha_discovery_prefix"] = settings.haDiscoveryPrefix;

  doc["ping1_warn_dist"] = settings.ping1WarnDist;
  doc["ping1_stop_dist"] = settings.ping1StopDist;
  doc["ping2_warn_dist"] = settings.ping2WarnDist;
  doc["ping2_stop_dist"] = settings.ping2StopDist;
  doc["ping_speed_ms"] = settings.pingSpeedMS;
  doc["dht_read_interval_seconds"] = settings.dhtReadIntervalSeconds;

  // Write a temporary file, then rename it over the real one. LittleFS renames
  // are atomic, so a power cut leaves either the old or the new settings,
  // never an empty file.
  File file = LittleFS.open(CONFIG_TEMP_PATH, "w");
  if (!file)
  {
    Log.errorln("Failed to open config file for writing");
    return false;
  }
  bool ok = serializeJson(doc, file) > 0;
  file.close();

  if (!ok || !LittleFS.rename(CONFIG_TEMP_PATH, CONFIG_PATH))
  {
    Log.errorln("Failed to write config file");
    LittleFS.remove(CONFIG_TEMP_PATH);
    return false;
  }
  return true;
}

void EraseSettings()
{
  LittleFS.remove(CONFIG_PATH);
  settings = Settings();
}
