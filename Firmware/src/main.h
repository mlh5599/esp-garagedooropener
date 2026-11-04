#pragma once

#include <Arduino.h>

struct ClimateData;

// Snapshot of the device state for the web dashboard.
struct DeviceStatus
{
  const char *door;
  bool motion;
  const char *bay1; // nullptr until the first reading
  const char *bay2;
  int bay1Cm; // last raw distance: -1 = none yet, 0 = no echo
  int bay2Cm;
  bool parkingLive;
  bool mqttConfigured;
  bool mqttConnected;
};

void setup();
void loop();
void SetupPins();
void SetupOTA();
void SetupInterrupts();

// Boot modes
void StartSetupMode(unsigned long retryAfterMs);
void StartNormalMode();
bool WaitForWiFi(unsigned long timeoutMs);
String SetupHotspotName();

// Used by the web UI
DeviceStatus GetDeviceStatus();
void ApplyMQTTSettings();
void ApplySensorSettings();
void RequestRestart(unsigned long delayMs);
unsigned long UptimeSeconds();
// Clears what the device left retained under the given topics before it moves
// to another broker, base topic or discovery prefix, or is factory reset.
// removeDevice also deletes the device from Home Assistant. Disconnects from
// the broker; call ApplyMQTTSettings() afterwards to reconnect.
void ForgetMQTTTopics(const String &baseTopic, const String &discoveryPrefix, bool removeDevice);

// Door
void HandleDoorToggle();
void HandleDoorOpen();
void HandleDoorClose();
void DoorToggleTimerCallback(void *pArg);
void HandleDoorState();
void ReadDoorState();

// Sensors
void HandleMotion();
void HandlePing();
// Fast parking readings for the setup page; stays on while this keeps being
// called, and ends a few seconds after the last call.
void StartParkingLiveReadings();
// Parking assist is on (interval > 0); when off it's left out of Home Assistant.
bool ParkingEnabled();
bool ParkingLiveActive();

// MQTT / Home Assistant
void MQTTCallback(char *topic, byte *payload, unsigned int length);
void MQTTReconnect();
void publishMQTT(const char *topic, const char *message);
void PublishClimate(const ClimateData &data);
void PublishDiagnostics();
void PublishAllStates();
void AnnounceToHomeAssistant();
