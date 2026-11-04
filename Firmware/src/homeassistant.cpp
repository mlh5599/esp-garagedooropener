#include "homeassistant.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <ESP8266WiFi.h>

// Home Assistant MQTT device discovery:
// https://www.home-assistant.io/integrations/mqtt/#device-discovery-payload
//
// Topics used by the entities (relative to the configured base topic):
//   <base>               ONLINE / OFFLINE (availability, also the MQTT last will)
//   <base>door           Open / Closed / Opening / Closing / Unknown
//   <base>doorcommand    OPEN / CLOSE / TOGGLE
//   <base>motion         Motion Detected / Clear
//   <base>temperature    degrees F
//   <base>humidity       percent
//   <base>heatindex      degrees F
//   <base>bay1/parkingstate, <base>bay2/parkingstate   NORMAL / WARN / STOP / UNKNOWN
//   <base>rssi           Wi-Fi signal in dBm
//   <base>uptime         seconds since boot

String HADeviceId()
{
  char id[24];
  snprintf(id, sizeof(id), "garagedoor_%06x", ESP.getChipId());
  return String(id);
}

String HAStatusTopic(const String &discoveryPrefix)
{
  return discoveryPrefix + "/status";
}

static String HADiscoveryTopic(const String &discoveryPrefix)
{
  return discoveryPrefix + "/device/" + HADeviceId() + "/config";
}

static JsonObject AddComponent(JsonObject cmps, const String &deviceId, const char *key,
                               const char *platform, const char *name)
{
  JsonObject c = cmps[key].to<JsonObject>();
  c["p"] = platform;
  c["name"] = name;
  c["unique_id"] = deviceId + "_" + key;
  return c;
}

static void AddParkingSensor(JsonObject cmps, const String &deviceId, const String &baseTopic,
                             const char *key, const char *name, const char *bay)
{
  JsonObject c = AddComponent(cmps, deviceId, key, "sensor", name);
  c["state_topic"] = baseTopic + bay + "/parkingstate";
  c["device_class"] = "enum";
  JsonArray options = c["options"].to<JsonArray>();
  options.add("NORMAL");
  options.add("WARN");
  options.add("STOP");
  c["icon"] = "mdi:car-brake-alert";
  // UNKNOWN (door not open, or no echo) shows as unknown in Home Assistant.
  c["value_template"] = "{{ 'None' if value == 'UNKNOWN' else value }}";
}

static JsonObject AddClimateSensor(JsonObject cmps, const String &deviceId, const String &baseTopic,
                                   const char *key, const char *name, const char *deviceClass,
                                   const char *unit, unsigned long expireAfter)
{
  JsonObject c = AddComponent(cmps, deviceId, key, "sensor", name);
  c["device_class"] = deviceClass;
  c["state_class"] = "measurement";
  c["unit_of_measurement"] = unit;
  c["state_topic"] = baseTopic + key;
  // Shows the sensor as unavailable if the DHT stops answering.
  if (expireAfter > 0)
    c["expire_after"] = expireAfter;
  return c;
}

bool PublishHADiscovery(PubSubClient &client, const String &baseTopic, const String &discoveryPrefix,
                        unsigned long climateIntervalSeconds, bool parkingEnabled)
{
  const String deviceId = HADeviceId();
  JsonDocument doc;

  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = deviceId;
  dev["name"] = "Garage Door Opener";
  dev["mf"] = "DIY";
  dev["mdl"] = "ESP-12E Garage Door Opener";
  dev["sw"] = FIRMWARE_VERSION;
  dev["cu"] = "http://" + WiFi.localIP().toString() + "/";

  JsonObject origin = doc["o"].to<JsonObject>();
  origin["name"] = "esp-garagedooropener";
  origin["sw"] = FIRMWARE_VERSION;
  origin["url"] = "https://github.com/mlh5599/esp-garagedooropener";

  // Shared by every entity: the device's ONLINE/OFFLINE last-will topic.
  doc["availability_topic"] = baseTopic;
  doc["payload_available"] = "ONLINE";
  doc["payload_not_available"] = "OFFLINE";

  JsonObject cmps = doc["cmps"].to<JsonObject>();
  const String commandTopic = baseTopic + "doorcommand";

  JsonObject door = AddComponent(cmps, deviceId, "door", "cover", "Door");
  door["device_class"] = "garage";
  door["state_topic"] = baseTopic + "door";
  door["command_topic"] = commandTopic;
  door["payload_open"] = "OPEN";
  door["payload_close"] = "CLOSE";
  door["payload_stop"] = nullptr; // no stop support; hides the stop button
  door["state_open"] = "Open";
  door["state_closed"] = "Closed";
  door["state_opening"] = "Opening";
  door["state_closing"] = "Closing";
  // "None" puts the cover into HA's unknown state.
  door["value_template"] = "{{ 'None' if value == 'Unknown' else value }}";

  JsonObject toggle = AddComponent(cmps, deviceId, "toggle", "button", "Toggle");
  toggle["command_topic"] = commandTopic;
  toggle["payload_press"] = "TOGGLE";
  toggle["icon"] = "mdi:garage-variant";

  JsonObject motion = AddComponent(cmps, deviceId, "motion", "binary_sensor", "Motion");
  motion["device_class"] = "motion";
  motion["state_topic"] = baseTopic + "motion";
  motion["payload_on"] = "Motion Detected";
  motion["payload_off"] = "Clear";

  // Allow two missed readings before a value counts as stale.
  unsigned long expireAfter = climateIntervalSeconds > 0 ? max(climateIntervalSeconds * 3, 60UL) : 0;
  AddClimateSensor(cmps, deviceId, baseTopic, "temperature", "Temperature", "temperature", "°F", expireAfter);
  AddClimateSensor(cmps, deviceId, baseTopic, "humidity", "Humidity", "humidity", "%", expireAfter);
  AddClimateSensor(cmps, deviceId, baseTopic, "heatindex", "Heat index", "temperature", "°F", expireAfter);

  if (parkingEnabled)
  {
    AddParkingSensor(cmps, deviceId, baseTopic, "bay1_parking", "Bay 1 parking", "bay1");
    AddParkingSensor(cmps, deviceId, baseTopic, "bay2_parking", "Bay 2 parking", "bay2");
  }
  else
  {
    // A component with only its platform removes that entity from the device.
    cmps["bay1_parking"]["p"] = "sensor";
    cmps["bay2_parking"]["p"] = "sensor";
  }

  JsonObject rssi = AddComponent(cmps, deviceId, "rssi", "sensor", "Wi-Fi signal");
  rssi["device_class"] = "signal_strength";
  rssi["state_class"] = "measurement";
  rssi["unit_of_measurement"] = "dBm";
  rssi["entity_category"] = "diagnostic";
  rssi["state_topic"] = baseTopic + "rssi";

  JsonObject uptime = AddComponent(cmps, deviceId, "uptime", "sensor", "Uptime");
  uptime["device_class"] = "duration";
  uptime["state_class"] = "total_increasing";
  uptime["unit_of_measurement"] = "s";
  uptime["entity_category"] = "diagnostic";
  uptime["state_topic"] = baseTopic + "uptime";

  String payload;
  serializeJson(doc, payload);
  const String topic = HADiscoveryTopic(discoveryPrefix);

  // Stream the payload so we don't need to enlarge PubSubClient's 256 byte buffer.
  bool ok = client.beginPublish(topic.c_str(), payload.length(), true) &&
            client.write((const uint8_t *)payload.c_str(), payload.length()) == payload.length() &&
            client.endPublish();

  if (ok)
    Log.noticeln("Published Home Assistant discovery to %s (%d bytes)", topic.c_str(), payload.length());
  else
    Log.errorln("Failed to publish Home Assistant discovery to %s", topic.c_str());
  return ok;
}

bool ClearHADiscovery(PubSubClient &client, const String &discoveryPrefix)
{
  // An empty retained message removes the device and clears the stored config.
  const String topic = HADiscoveryTopic(discoveryPrefix);
  bool ok = client.publish(topic.c_str(), "", true);
  Log.noticeln("Removed Home Assistant discovery from %s", topic.c_str());
  return ok;
}
