#include "climate.h"
#include <pinDefinitions.h>

static DHT dht(DHT_PIN, DHTTYPE);
static unsigned long lastSensorCheckMillis = 0;

unsigned long dhtReadIntervalMillis = 0;
ClimateData lastClimate = {NAN, NAN, NAN};
bool lastClimateValid = false;

void SetupDHT()
{
  dht.begin();
}

static ClimateData ReadFromDHT()
{
  // A DHT22 read takes about 250ms and may be up to 2 seconds old.
  ClimateData data;
  data.humidity = dht.readHumidity();
  data.temperatureF = dht.readTemperature(true);
  data.heatIndexF = dht.computeHeatIndex(data.temperatureF, data.humidity);
  return data;
}

bool CheckDHT(ClimateData &data)
{
  if (dhtReadIntervalMillis == 0)
    return false;
  if (lastSensorCheckMillis != 0 && millis() - lastSensorCheckMillis < dhtReadIntervalMillis)
    return false;

  lastSensorCheckMillis = millis();
  ClimateData reading = ReadFromDHT();
  if (isnan(reading.humidity) || isnan(reading.temperatureF))
    return false;

  data = reading;
  lastClimate = reading;
  lastClimateValid = true;
  return true;
}
