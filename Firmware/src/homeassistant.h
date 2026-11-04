#ifndef HOMEASSISTANT_H
#define HOMEASSISTANT_H

#include <Arduino.h>
#include <PubSubClient.h>

#define FIRMWARE_VERSION "1.2.0"

// Stable per-board ID built from the ESP chip ID, e.g. "garagedoor_1a2b3c".
String HADeviceId();

// The topic Home Assistant publishes "online"/"offline" to when it starts or stops.
String HAStatusTopic(const String &discoveryPrefix);

// Publishes one retained MQTT device-discovery message that creates the
// garage door device and all of its entities in Home Assistant.
// baseTopic must end with '/' (same convention as the rest of the firmware).
// climateIntervalSeconds sets when stale climate readings expire (0 = never).
// With parkingEnabled false the parking sensors are removed from the device.
bool PublishHADiscovery(PubSubClient &client, const String &baseTopic, const String &discoveryPrefix,
                        unsigned long climateIntervalSeconds, bool parkingEnabled);

// Removes the device and its entities from Home Assistant.
bool ClearHADiscovery(PubSubClient &client, const String &discoveryPrefix);

#endif // HOMEASSISTANT_H
