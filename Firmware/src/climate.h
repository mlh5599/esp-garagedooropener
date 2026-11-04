#ifndef CLIMATE_H
#define CLIMATE_H

#include <DHT.h>

#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

struct ClimateData {
  float humidity;
  float temperatureF;
  float heatIndexF;
};

extern unsigned long dhtReadIntervalMillis; // 0 = sensor off
extern ClimateData lastClimate;
extern bool lastClimateValid;

void SetupDHT();
// Reads the sensor when the read interval has elapsed. Returns true and fills
// `data` only when a fresh, valid reading was taken.
bool CheckDHT(ClimateData &data);

#endif // CLIMATE_H
