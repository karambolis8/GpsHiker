// dodać obsługe dodatkowego buttona do resetu (początek wędrówki - liczenie czasu, wysokości, zejść i podejść)

// sprawdzic czy button zle sie zachowuje tez jak go zastapic stykaniem kabelkow (moze zle siedzi w plytce)

// sync GPS readout 100ms (policzyć serial monitorem)

// liczenie drogi
// - https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Location.md
// - https://github.com/SlashDevin/NeoGPS/issues/15

// napięcie bateryjki z mryganiem przy 2.7V

//obudowa z wystawieniem USB C ładowania
//USB mini dostepne do programowania po odkreceniu srubek arduino
//18650 montowane na trytki
//wstawienie jakies TMP sensora na krawedzi obudowy
//wentylacja jaklas dla BMP z siateczka (mozna zrobic motyw z pause print i podlozeniem siateczki np z herbaty do zadrukowania, albo po prostu wkleić na CA)
//jakies narozniki czy cos takiego z TPU
//jakas petelka na sznurek/karabinczyk
//jakis klips do ubrania

#include "Config.h"

#include "Utils.h"
#include "Temp.h"
#include "BME280.h"
#include "TempOled.h"
#include "BME280Oled.h"
#include "Button.h"
#include "GPS.h"
#include <Wire.h>
#include <U8x8lib.h>

struct TemperatureSensor temperatureReadouts;
struct Bme280Sensor bme280SensorReadouts;
struct GpsReadouts gpsReadouts;

float volts = 0.0;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
unsigned long lastScreenUpdate = 0; 
bool doScreenUpdate = false;
int previousScreen = -1;
int currentScreen = 1;
bool blink = true;

void setup()
{
  initOled();
  printHeader();

  u8x8.setCursor(0,2);
  u8x8.print(F("Initializing GPS"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initGPS();

  u8x8.setCursor(0,3);
  u8x8.print(F("Initializing BME")); 
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initBme();
  initTempSensor();
  initButton();
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
    currentScreen = ((currentScreen + 1) % 3 ) +1;
  }

  unsigned long now = millis();
  
  if ( now - lastScreenUpdate < OLED_REFRESH ) {
    doScreenUpdate = false;
  } else {
    doScreenUpdate = true;
    lastScreenUpdate = now;
  }
  
  readGPS();
  performReadouts(); 

  if(doScreenUpdate)
  {
    updateScreen();
    lastScreenUpdate = millis();
  }
}

void updateScreen()
{
  printHeader();
//6 lines

//1. Current readouts
// - Bar alt (przewyższenie)
// - GPS alt (nad poziomem morza)
// - Temperature + Humidity
// - path kilometers
// - lat lon short

//2. GPS
// - Lat
// - Lon
// - Heading (stopnie + NWES)
// - GPS alt (nad poziomem morza)
// - GPS speed
// - numsat

//3. Stats
// - max wysokosc bar
// - max wysokość gps
// - max temp + min temp
// - czas od resetu
// - path kilometers

  if(currentScreen == 2)
  {
    displayStatistics();
  }
  else if(currentScreen == 1)
  {
    displayCurrentReadouts();
  }

  u8x8.setCursor(15,7);
  if(blink)
    u8x8.print(".");
  else
    u8x8.print(" ");
  blink = not blink;
}

void printHeader()
{
  u8x8.setInverseFont(1);
  u8x8.setCursor(0,0);

  if(wasGpsFix == false && gpsHasFix == false)
  {
    u8x8.print(F("Waiting for sats"));
  }
  else
  {
    u8x8.print(year);  
    u8x8.print(F("."));
    if(month < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(month);
    u8x8.print(F("."));
    if(day < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(day);
    u8x8.print(F("  "));
    u8x8.setCursor(11, 0);
    if(hour < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(hour);
    u8x8.print(F(":"));
    if(minutes < 10)
    {
      u8x8.print(0);      
    }
    u8x8.print(minutes);
  }

  u8x8.setInverseFont(0);

  u8x8.setCursor(0, 1);

  if(gpsHasFix)
  {
    u8x8.print(F("Fix   "));
  }
  else
  {    
    u8x8.print(F("No fix"));
  }
  
  
  u8x8.setCursor(7, 1);
  u8x8.print(F("Batt:"));
  u8x8.setCursor(12, 1);
  u8x8.print(volts, 1);
  u8x8.setCursor(15, 1);
  u8x8.print(F("v"));
}

void displayCurrentReadouts()
{
  if(currentScreen != previousScreen)
  {
    clearLines(2);
    displayCurrentReadoutsLayout();
    previousScreen = currentScreen;
  }

  u8x8.setCursor(6, 2);
  u8x8.print(year);  
  u8x8.print(F("."));
  u8x8.print(month);
  u8x8.print(F("."));
  u8x8.print(day);

  u8x8.setCursor(6, 6);
  u8x8.print(hour);
  u8x8.print(F(":"));
  u8x8.print(minutes);

  u8x8.setCursor(8, 3);
  if(Speed <= 99)
    u8x8.print(FS(space));
  if(Speed <= 9)
    u8x8.print(FS(space));
  u8x8.print(Speed);

  displayCurrentTemp(u8x8, T1);
  displayBmeAlt(u8x8, PressureAltitude);
  displayBmeHumidity(u8x8, Humidity);
}

void displayCurrentReadoutsLayout()
{ 
  displayCurrentGpsData();
  displayCurrentTempLayout(u8x8);
  displayCurrentBmeAltLayout(u8x8);
  displayCurrentBmeHumidityLayout(u8x8);
}

void displayCurrentGpsData() //U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0, 2);
  u8x8.print(F("Date:"));

  u8x8.setCursor(0, 6);
  u8x8.print(F("Time:"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));
}

void displayStatistics()
{
  if(currentScreen != previousScreen)
  {
    clearLines(1);
    displayStatisticsLayout();
    previousScreen = currentScreen;
  }

  u8x8.setCursor(6, 2);
  u8x8.print(year);  
  u8x8.print(F("."));
  u8x8.print(month);
  u8x8.print(F("."));
  u8x8.print(day);
  
  u8x8.setCursor(8, 3);
  if(MaxSpeed <= 99)
    u8x8.print(FS(space));
  if(MaxSpeed <= 9)
    u8x8.print(FS(space));
  u8x8.print(MaxSpeed);

  displayMaxTemp(u8x8, T1Max);
  displayBmeAlt(u8x8, MaxPressureAltitude);
  displayBmeHumidity(u8x8, MaxHumidity);
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

void performReadouts()
{
  calculateTemp(&temperatureReadouts);
  calculatePressureHumidity(&bme280SensorReadouts);
  calculateBattery();
}

void calculateBattery()
{
  volts = 4.2;
}