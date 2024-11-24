#include "Config.h"

const float VccReference = 5.0;

#ifdef GPS_BAUD
  #include "NMEAGPS.h"
  #include "GPSport.h"
  
  #ifndef NMEAGPS_PARSE_GSV
    #error You must define NMEAGPS_PARSE_GSV in NMEAGPS_cfg.h!
  #endif
  
  #ifndef NMEAGPS_PARSE_SATELLITES
    #error You must define NMEAGPS_PARSE_SATELLITE in NMEAGPS_cfg.h!
  #endif
  
  #ifndef NMEAGPS_PARSE_SATELLITE_INFO
    #error You must define NMEAGPS_PARSE_SATELLITE_INFO in NMEAGPS_cfg.h!
  #endif

  NMEAGPS  gps;
  gps_fix  fix;
  long Speed, MaxSpeed = 0;
  int numSV = 0;  
#endif

#ifdef OLED
  #define FS(x) (__FlashStringHelper*)(x)
  
  #include <Wire.h>
  #include <Arduino.h>
  #include <U8x8lib.h>
  
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
  unsigned long lastScreenUpdate = 0; 
  bool doScreenUpdate = false;
  int previousScreen = -1;
  int currentScreen = 1;

  const char clearLine[] PROGMEM = { "                " };
  const char space[] PROGMEM = { " " };

  bool blink = true;
#endif

#ifdef BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter pressAltFilter(0.03, 0.003, 0.03);

  Adafruit_BME280 bme;
  int PressureAltitude, MaxPressureAltitude = 0;
  float gndLevelPressure = 0;
  int avgSize = 10;
#endif

#ifdef TEMP
  #include <SimpleKalmanFilter.h>  
  SimpleKalmanFilter temp1Filter(1, 1, 0.01);

  unsigned long lastTempReadout = 0;
  int T1, T1Max = 0;
  int tempPin = 9;
#endif

#ifdef Sensor_DS18B20
  #include <DS18B20.h>
  DS18B20 ds(tempPin);
  uint8_t address[] = {0x28, 0xD, 0x6A, 0x2, 0x26, 0x20, 0x1, 0x25};
  uint8_t selected;
#endif

unsigned long lastPerformedReadouts = 0;

void setup()
{  
#ifdef OLED
  initOled();
  displayHeader();
#endif

#ifdef GPS_BAUD
#ifdef OLED
  u8x8.setCursor(0,2);
  u8x8.print(F("Initializing GPS"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  initGPS();
#endif

#ifdef BME280
#ifdef OLED
  u8x8.setCursor(0,3);
  u8x8.print(F("Initializing BME"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  initBme();
#endif

#ifdef DS1820
  selected = ds.select(address);
#endif

#ifdef BUTTON_INPUT
  initButton();
#endif
}

#ifdef OLED
void initOled()
{  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}
#endif

#ifdef BUTTON_INPUT
void initButton()
{
  pinMode(BUTTON_INPUT,INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), buttonPressed, HIGH);
}

void buttonPressed()
{   
  currentScreen = ((currentScreen + 1) % 3 ) +1;
}
#endif

#ifdef BME280
void initBme()
{
  bme.begin(0x76);

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

void calculatePressAlt()
{
  float alt = bme.readAltitude(gndLevelPressure);
  
  if(isnan(alt))
    return;
  
  PressureAltitude = (int)pressAltFilter.updateEstimate(alt);
  
  if(PressureAltitude > MaxPressureAltitude)
    MaxPressureAltitude = PressureAltitude;
}
#endif


void loop()
{
  unsigned long now = millis();
#ifdef OLED
  if ( now - lastScreenUpdate < OLED_REFRESH ) {
    doScreenUpdate = false;
  } else {
    doScreenUpdate = true;
    lastScreenUpdate = now;
  }
#endif
  
#ifdef GPS_BAUD
    readGPS();
#endif

  performReadouts();
  lastPerformedReadouts = millis();  

#ifdef OLED
  if(doScreenUpdate)
  {
    updateScreen();
    lastScreenUpdate = millis();
  }
#endif
}

#ifdef OLED
void updateScreen()
{
#ifdef GPS_BAUD
  if (numSV < GPS_MIN_SAT)
  {
    if(currentScreen != previousScreen)
    {
      printWaitingLayout();
      currentScreen = previousScreen = 0;
    }
    u8x8.setCursor(5,2);
    if(numSV < 10)
      u8x8.print(FS(space));
    u8x8.print(numSV);
  }
  else
  {
#endif
    if(currentScreen == 1)
    {
      displayStatistics();
    }
    else if(currentScreen == 2)
    {
      displayCurrentReadouts();
    }
#ifdef GPS_BAUD
  }
#endif

  u8x8.setCursor(15,7);
  if(blink)
    u8x8.print(".");
  else
    u8x8.print(" ");
  blink = not blink;
}

void printWaitingLayout()
{
    clearLines(1);
    u8x8.setCursor(0, 1);
    u8x8.print(F("Waiting for sats"));
    u8x8.setCursor(0, 2);
    u8x8.print(F("Sats:"));
}

void displayHeader()
{
  u8x8.setCursor(0,0);
  u8x8.print(FS(clearLine));
  u8x8.setInverseFont(1);
  u8x8.print(F("== GPS Logger =="));  
  u8x8.setInverseFont(0);
}

void displayCurrentReadouts()
{
  if(currentScreen != previousScreen)
  {
    clearLines(1);
    displayCurrentReadoutsLayout();
    previousScreen = currentScreen;
  }
    
#ifdef GPS_BAUD
  u8x8.setCursor(5, 1);
  if(numSV < 10)
    u8x8.print(FS(space));
  u8x8.print(numSV);

  u8x8.setCursor(8, 3);
  if(Speed <= 99)
    u8x8.print(FS(space));
  if(Speed <= 9)
    u8x8.print(FS(space));
  u8x8.print(Speed);
#endif

#ifdef TEMP
  displayCurrentTemp();
#endif

#ifdef BME280
  u8x8.setCursor(8,5);
  if(PressureAltitude <= 999)
    u8x8.print(FS(space));
  if(PressureAltitude <= 99)
    u8x8.print(FS(space));
  if(PressureAltitude <= 9)
    u8x8.print(FS(space));
  u8x8.print(PressureAltitude);
#endif
}

void displayCurrentReadoutsLayout()
{    
#ifdef GPS_BAUD
  u8x8.setCursor(0, 1);
  u8x8.print(F("Sats:"));
  
  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));
#endif

#ifdef TEMP
  displayCurrentTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Bar Alt:"));
  u8x8.setCursor(12,5);
  u8x8.print(F("m"));
#endif
}

void displayStatistics()
{
  if(currentScreen != previousScreen)
  {
    clearLines(1);
    displayStatisticsLayout();
    previousScreen = currentScreen;
  }

#ifdef GPS_BAUD
  if(currentScreen == 2)
  {
    clearLines(1);
    displayStatisticsLayout();  
  }

  u8x8.setCursor(5, 1); 
  if(numSV < 10)
    u8x8.print(FS(space));
  u8x8.print(numSV);
  
  u8x8.setCursor(8, 3);
  if(MaxSpeed <= 99)
    u8x8.print(FS(space));
  if(MaxSpeed <= 9)
    u8x8.print(FS(space));
  u8x8.print(MaxSpeed);
#endif

#ifdef TEMP
  displayMaxTemp();
#endif

#ifdef BME280
  u8x8.setCursor(8,5);  
    if(MaxPressureAltitude <= 999)
    u8x8.print(FS(space));
  if(MaxPressureAltitude <= 99)
    u8x8.print(FS(space));
  if(MaxPressureAltitude <= 9)
    u8x8.print(FS(space));
  u8x8.print(MaxPressureAltitude);
#endif
}

#ifdef OLED
void displayStatisticsLayout()
{
  #ifdef GPS_BAUD
  u8x8.setCursor(0, 1); 
  u8x8.print(F("Sats:"));
  
  u8x8.setCursor(0, 3);
  u8x8.print(F("Max Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));
#endif

#ifdef TEMP
  displayMaxTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Max Alt:"));
    u8x8.setCursor(12, 5);
    u8x8.print(F("m"));
#endif
}
#endif

#ifdef TEMP
void displayCurrentTemp()
{
  u8x8.setCursor(5, 7);  
  if((T1 <= 99 && T1 > 0) || (T1 < 0 && abs(T1) <= 9))
    u8x8.print(FS(space));
  if(T1 > 0 && T1 <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1);
}

void displayCurrentTempLayout()
{
  u8x8.setCursor(0, 7);
  u8x8.print(F("Tmp:"));
  u8x8.setCursor(8, 7);  
  u8x8.print(F("C"));
}

void displayMaxTemp()
{
  u8x8.setCursor(9, 7);
  if((T1Max <= 99 && T1Max > 0) || (T1Max < 0 && abs(T1Max) <= 9))
    u8x8.print(FS(space));
  if(T1Max > 0 && T1Max <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1Max);
}

void displayMaxTempLayout()
{
  u8x8.setCursor(0, 7);
  u8x8.print(F("Max Tmp:"));
  u8x8.setCursor(12, 7); 
  u8x8.print(F("C"));
}
#endif


void clearLines(int startingLine)
{
    for(startingLine; startingLine < 8; startingLine++)
    {
      u8x8.setCursor(0,startingLine);
      u8x8.print(FS(clearLine));
    }
}
#endif

#ifdef GPS_BAUD

void initGPS()
{
  gpsPort.begin(GPS_BAUD);
}

void readGPS()
{
  while (gps.available( gpsPort )) 
  {
    fix = gps.read();
    numSV = gps.sat_count;
    
    if(fix.valid.speed)
    {
      Speed = fix.speed_kph();
    }
  }
}

void calculateGps()
{
  if (numSV >= GPS_MIN_SAT && Speed > MaxSpeed)
  {
    MaxSpeed = Speed;
  }
}
#endif

void performReadouts()
{
#ifdef GPS_BAUD
  calculateGps();
#endif

#ifdef TEMP
  calculateTemp();
#endif

#ifdef BME280
  calculatePressAlt();
#endif
}


#ifdef TEMP
void calculateTemp()
{
  if(millis() - lastTempReadout >= ANALOG_READ_DELAY)
  {
    T1 = temp1Filter.updateEstimate(calculateRawTemp(tempPin));
    if(T1 > T1Max)
      T1Max = T1;

    lastTempReadout = millis();
  }
}

#ifdef Sensor_DS18B20
int calculateRawTemp(int port)
{
  if(selected) {
    return (int)ds.getTempC();
  }
}
#endif

#ifdef LM35
int calculateRawTemp(int port)
{
  int readVal = analogRead(port);
  float volt = readVal * VccReference / 1024.0;
  return volt * 100.0;
}
#endif

#ifdef TMP36
int calculateRawTemp(int port)
{
  int readVal = analogRead(port);
  float volt = readVal * VccReference / 1024.0 ;
  return (volt - 0.5) * 100.0;
}
#endif
#endif
