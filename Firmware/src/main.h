#include <Arduino.h>

ICACHE_RAM_ATTR void MotionDetected();
ICACHE_RAM_ATTR void DoorOpenStateChanged();
ICACHE_RAM_ATTR void DoorClosedStateChanged();
void HandleDoorToggle();
void DoorToggleTimerCallback();
void PublishDoorState();

void MQTTCallback(char *topic, byte *payload, unsigned int length);
void MQTTReconnect();
void publishMQTT(const char *topic, const char *message);

void SetupDHT();
void HandleDHT();
void ReadDHT();

void setup();
void loop();
void SetupPins();
void SetupWiFi();
void SetupOTA();
void SetupMQTT();
void SetupInterrupts();

void APModeSetup();
void StationModeSetup();
void APLoop();

void handleWiFiSetup();
void handleWiFiSetupComplete();
void handleRoot();
void handleMQTTConfig();
void handleMQTTConfigComplete();
void handleDeviceConfig();
void handleDeviceConfigComplete();

void HandlePing();
void HandleMotion();
void HandleDoorState();
void ReadDoorState();