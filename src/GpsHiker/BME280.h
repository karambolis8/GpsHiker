#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SimpleKalmanFilter.h>

struct Bme280Sensor
{
  int pressureAltitude;
  int maxPressureAltitude;
  int humidity;
  int maxHumidity;
  int minHumidity;
};

SimpleKalmanFilter pressAltFilter(1, 1, 0.01);

Adafruit_BME280 bme;
float gndLevelPressure = 0;
int avgSize = 10;


void initBme()
{
  bme.begin(BME280_Address);

  float gndLeveLAverage = 0.0;
  
  for(int i = 0; i < avgSize; i++)
  {
    float sensorRawPressure = bme.readPressure();
    if(isnan(sensorRawPressure))
      i -= 1;
    else
      gndLeveLAverage += sensorRawPressure;
  }
  
  gndLevelPressure = gndLeveLAverage / avgSize * 0.01;
}

int calculateBmeAlt()
{
  float alt = bme.readAltitude(gndLevelPressure);
  return (int)pressAltFilter.updateEstimate(alt);
}

int calculateBmeHumidity()
{
  float h = bme.readHumidity();
  return (int)h;
}

void calculatePressureHumidity(Bme280Sensor* readouts)
{
  readouts->pressureAltitude = calculateBmeAlt(); 
  
  if(readouts->pressureAltitude > readouts->maxPressureAltitude)
    readouts->maxPressureAltitude = readouts->pressureAltitude;

  readouts->humidity = calculateBmeHumidity();

  if(readouts->humidity > readouts->maxHumidity)
    readouts->maxHumidity = readouts->humidity;

  if(readouts->humidity < readouts->minHumidity)
    readouts->minHumidity = readouts->humidity;
}