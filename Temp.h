const float VccReference = 5.0;

#ifdef Sensor_DS18B20

#include <DS18B20.h>

#define SensorDigitalPin 9

uint8_t address[] = DS18B20_Address;
uint8_t selected;

DS18B20 ds(SensorDigitalPin);

void initTempSensor()
{
  selected = ds.select(address);
}

int calculateRawTemp()
{
  if(selected) {
    float temp = ds.getTempC();
    return (int)temp;
  }
  
  return 0;
}
#endif

#ifdef LM35
#define SensorAnalogPin 0

void initTempSensor() { }

int calculateRawTemp()
{
  int readVal = analogRead(SensorAnalogPin);
  float volt = readVal * VccReference / 1024.0;
  return volt * 100.0;
}
#endif

#ifdef TMP36
#define SensorAnalogPin 0

void initTempSensor() { }

int calculateRawTemp()
{
  int readVal = analogRead(SensorAnalogPin);
  float volt = readVal * VccReference / 1024.0 ;
  return (volt - 0.5) * 100.0;
}
#endif