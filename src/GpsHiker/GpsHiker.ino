// TODO:
// 1. reset button support (reseting bar alt, trip time & trip distance)
// 2. check if screen button works properly when shotring two wires instead of tact switch pressing
// 3. implement GPS path calculation
// 4. implement stats & readouts screen
// 5. refactor OLED refresh
// 6. reset trip screen that waits for GPS fix before seting initial values

// TODO case:
// design 3d-printed case with features:
//   - USB C charging
//   - programming port available when uscrewing the arduino
//   - 18650 or small lipo battery holder
//   - TMP sensor on the edge of the case
//   - ventilation for BME280
//   - some kind of clip for attaching to backpack
//   - carabiner or string loop
//   - TPU corners for protection


#include "Config.h"

#include "Utils.h"
#include "Temp.h"
#include "BME280.h"
#include "TempOled.h"
#include "BME280Oled.h"
#include "Button.h"
#include "GPS.h"
#include "GPSOled.h"
#include <Wire.h>
#include <U8x8lib.h>
#include "GpsHikerModels.h"
#include "Battery.h"

struct TemperatureSensor temperatureReadouts;
struct Bme280Sensor bme280SensorReadouts;
struct GpsReadouts gpsReadouts;
struct BatteryMonitor battery = { 4.2, false };
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
struct ScreenUpdate screenUpdate = {0, false, -1, 0, true};

void setup()
{
  Serial.begin(115200);

  initOled();
  initButton();
  printHeader();

  u8x8.setCursor(0,2);
  u8x8.print(F("Initializing GPS"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initGPS();

  u8x8.setCursor(0,3);
  u8x8.print(F("Initializing BME")); 
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initBme();
  
  u8x8.setCursor(0,4);
  u8x8.print(F("Initializing TMP")); 
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initTempSensor();
}

void initOled()
{  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void loop()
{
  if(readButton())
  {
    screenUpdate.currentScreen = ((screenUpdate.currentScreen + 1) % 3);
  }
  
  performReadouts(); 

  unsigned long now = millis();
  
  if ( now - screenUpdate.lastScreenUpdate < OLED_REFRESH ) {
    screenUpdate.doScreenUpdate = false;
  } else {
    screenUpdate.doScreenUpdate = true;
    screenUpdate.lastScreenUpdate = now;
  }

  if(screenUpdate.doScreenUpdate)
  {
    updateScreen();
    screenUpdate.lastScreenUpdate = millis();
  }
}

void performReadouts()
{
  readGPS(&gpsReadouts);
  calculateTemp(&temperatureReadouts);
  calculatePressureHumidity(&bme280SensorReadouts);
  calculateBattery(&battery);
}

void updateScreen()
{
  printHeader();

  if(screenUpdate.currentScreen == 0)
  {
    displayCurrentReadouts();
  }
  else if(screenUpdate.currentScreen == 1)
  {
    displayGpsScreen();
  }
  else
  {
    displayStatistics();
  }

  u8x8.setCursor(15,7);
  if(screenUpdate.blink)
    u8x8.print(".");
  else
    u8x8.print(" ");
  screenUpdate.blink = not screenUpdate.blink;
}

void printHeader()
{
  u8x8.setInverseFont(1);
  u8x8.setCursor(0,0);

  if(gpsReadouts.wasGpsFix == false && gpsReadouts.gpsHasFix == false)
  {
    u8x8.print(F("Waiting for sats"));
  }
  else
  {
    u8x8.print(gpsReadouts.year);  
    u8x8.print(F("."));
    if(gpsReadouts.month < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(gpsReadouts.month);
    u8x8.print(F("."));
    if(gpsReadouts.day < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(gpsReadouts.day);
    u8x8.print(F("  "));
    u8x8.setCursor(11, 0);
    if(gpsReadouts.hour < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(gpsReadouts.hour);
    u8x8.print(F(":"));
    if(gpsReadouts.minutes < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(gpsReadouts.minutes);
  }

  u8x8.setInverseFont(0);

  u8x8.setCursor(0, 1);

  if(gpsReadouts.gpsHasFix)
  {
    u8x8.print(F("Fix   "));
  }
  else
  {    
    u8x8.print(F("No fix"));
  }

  printBattery(u8x8, &battery);
}

void displayCurrentReadouts()
{
//1. Current readouts
// - Bar alt (przewyższenie)
// - GPS alt (nad poziomem morza)
// - Temperature + Humidity
// - path kilometers
// - lat lon short

 if(screenUpdate.currentScreen != screenUpdate.previousScreen)
  {
    clearLines(2);
    displayCurrentReadoutsLayout();
    screenUpdate.previousScreen = screenUpdate.currentScreen;
  }

  u8x8.setCursor(0, 2);
  u8x8.print(F("BME Alt:"));
  u8x8.setCursor(9, 2);
  u8x8.print(bme280SensorReadouts.pressureAltitude);
  u8x8.print(F("m"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Alt:"));
  u8x8.setCursor(9, 3);
  u8x8.print(gpsReadouts.fix.alt.whole);
  u8x8.print(F("m"));

  u8x8.setCursor(0, 4);
  u8x8.print(F("Tmp:"));
  u8x8.print(temperatureReadouts.currentTemp);
  u8x8.print(F("C"));
  u8x8.setCursor(9, 4);
  u8x8.print(F("Hum:"));
  u8x8.print(bme280SensorReadouts.humidity);
  u8x8.print(F("%"));

  u8x8.setCursor(0, 5);
  u8x8.print(F("Trip: "));
  u8x8.print(F("17.3km"));

  u8x8.setCursor(0, 6);
  u8x8.print(F("Trip: "));
  u8x8.print(F("10:35h"));

  u8x8.setCursor(0, 7);
  u8x8.print(gpsReadouts.fix.latitude(), 3);
  u8x8.setCursor(7, 7);
  u8x8.print(gpsReadouts.fix.longitude(), 3);
}

void displayCurrentReadoutsLayout()
{ 
  // displayCurrentGpsDataLayout();
  // displayCurrentTempLayout(u8x8);
  // displayCurrentBmeAltLayout(u8x8);
  // displayCurrentBmeHumidityLayout(u8x8);
}

void displayGpsScreen()
{
  if(screenUpdate.currentScreen != screenUpdate.previousScreen)
  {
    clearLines(2);
    displayCurrentGpsDataLayout(u8x8);
    screenUpdate.previousScreen = screenUpdate.currentScreen;
  }

  displayCurrentGpsData(u8x8, &gpsReadouts);
}

void displayStatistics()
{
//3. Stats
// - wysokosc poczatkowa GPS
// - GPS max speed
// - max temp + min temp
// - max + min hum
// - trip time
// - trip km
// GPS XXXXm-XXXXm
// ODO XX:XXkm XX:Xh
  if(screenUpdate.currentScreen != screenUpdate.previousScreen)
  {
    clearLines(1);
    displayStatisticsLayout();
    screenUpdate.previousScreen = screenUpdate.currentScreen;
  }
  
  u8x8.setCursor(8, 3);
  if(gpsReadouts.maxSpeed <= 99)
    u8x8.print(FS(space));
  if(gpsReadouts.maxSpeed <= 9)
    u8x8.print(FS(space));
  u8x8.print(gpsReadouts.maxSpeed);

  displayMaxTemp(u8x8, temperatureReadouts.maxTemp);
  displayBmeAlt(u8x8, bme280SensorReadouts.maxPressureAltitude);
  displayBmeHumidity(u8x8, bme280SensorReadouts.maxHumidity);
}

void displayStatisticsLayout()
{
  u8x8.setCursor(0, 2);
  u8x8.print(F("Date:"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("Max Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));

  displayMaxTempLayout(u8x8);
  displayMaxBmeAltLayout(u8x8);
  displayMaxBmeHumidityLayout(u8x8);
}

void clearLines(int startingLine)
{
    for(startingLine; startingLine < 8; startingLine++)
    {
      u8x8.setCursor(0, startingLine);
      u8x8.print(FS(clearLine));
    }
}