#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

#define HA_DEFAULT_DISCOVERY_PREFIX "homeassistant"
#define DEFAULT_MQTT_BASE_TOPIC "garagedoor/"

// Everything persisted in /config.json on LittleFS.
struct Settings
{
  String wifiSsid;
  String wifiPassword;
  // True once the device has connected with the current Wi-Fi credentials.
  // Unverified credentials fall back to setup mode if they fail; verified
  // ones never do, so a router outage can't reopen the setup hotspot.
  bool wifiVerified = false;

  String mqttHost;
  int mqttPort = 1883;
  String mqttUser;
  String mqttPassword;
  String mqttBaseTopic = DEFAULT_MQTT_BASE_TOPIC;
  String haDiscoveryPrefix = HA_DEFAULT_DISCOVERY_PREFIX;

  // Parking distances are in cm. 0 disables the feature.
  unsigned int ping1WarnDist = 0;
  unsigned int ping1StopDist = 0;
  unsigned int ping2WarnDist = 0;
  unsigned int ping2StopDist = 0;
  unsigned long pingSpeedMS = 0;
  unsigned long dhtReadIntervalSeconds = 0;
};

extern Settings settings;

bool MountFileSystem();
bool LoadSettings();
bool SaveSettings();
// Deletes the config file; the device boots into setup mode afterwards.
void EraseSettings();
// Base topic always ends in '/', discovery prefix never does.
void NormalizeSettings();
bool HasWiFiSettings();

#endif // SETTINGS_H
