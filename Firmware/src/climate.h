#ifndef CLIMATE_H
#define CLIMATE_H

#include <DHT.h>

#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

struct ClimateData {
  float humidity;
  float temperatureF;
  float heatIndexF;
};

extern DHT dht;
extern int lastTemp;
extern int lastHumidity;
extern int lastHeatIndex;
extern unsigned long lastSensorCheckMillis;
extern bool sendTempUpdate;
extern bool sendHumidityUpdate;
extern bool sendHeatIndexUpdate;
extern unsigned long dhtReadIntervalMillis;

void SetupDHT();
ClimateData CheckDHT();
ClimateData ReadFromDHT();

#endif // CLIMATE_H