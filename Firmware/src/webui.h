#ifndef WEBUI_H
#define WEBUI_H

#include <Arduino.h>

// Scans for nearby Wi-Fi networks for the setup page (takes ~2 seconds).
// The station interface must be enabled (WIFI_STA or WIFI_AP_STA).
void WebUIScanNetworks();

// Setup mode: captive portal on the device's own hotspot with a single
// page for Wi-Fi and MQTT settings.
void WebUIBeginSetupMode(const String &hotspotName);

// Normal mode: status dashboard plus the settings pages.
void WebUIBeginNormalMode();

void WebUILoop();

#endif // WEBUI_H
