#include "climate.h"
#include <DHT.h>
#include <pinDefinitions.h>

//###########################################
//  DHT Defs
//###########################################
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
DHT dht(DHT_PIN, DHTTYPE);
int lastTemp = -99;
int lastHumidity = -99;
int lastHeatIndex = -99;
unsigned long lastSensorCheckMillis = 0;
bool sendTempUpdate = false;
bool sendHumidityUpdate = false;
bool sendHeatIndexUpdate = false;
unsigned long dhtReadIntervalMillis = 0;

//#############################################################
//  DHT
//#############################################################
void SetupDHT()
{
  dht.begin();
}

ClimateData CheckDHT()
{
  ClimateData data;
  if (dhtReadIntervalMillis > 0 &&
      (lastSensorCheckMillis + dhtReadIntervalMillis < millis() || lastSensorCheckMillis == 0))
  {
    data = ReadFromDHT();
    lastSensorCheckMillis = millis();
  }
  return data;
}


ClimateData ReadFromDHT()
{
  ClimateData data;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  data.humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  // float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  data.temperatureF = dht.readTemperature(true);

  // if (isnan(data.h) || isnan(data.f))
  // {
  //   publishMQTT("environment", "DHT Fail!");
  //   return data;
  // }

  // Compute heat index in Fahrenheit (the default)
  data.heatIndexF = dht.computeHeatIndex(data.temperatureF, data.humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  // float hic = dht.computeHeatIndex(t, h, false);

  return data;
}