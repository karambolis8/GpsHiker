#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SimpleKalmanFilter.h>

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
