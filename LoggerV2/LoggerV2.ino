//https://electronics.stackexchange.com/questions/25278/how-to-connect-multiple-i2c-interface-devices-into-a-single-pin-a4-sda-and-a5

#include "Config.h"

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
  int zeroingCounter = GPS_ZEROING_TIME;
  unsigned long lastZeroingUpdate = 0;
  long Speed = 0;
  long MaxSpeed = 0;
  int Height = 0;
  int MaxHeight, ZeroHeight = 0;
  int numSV = 0;
  
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter gpsAltitudeFilter(0.03, 0.003, 0.03); 
  
#endif

#ifdef OLED
  #define FS(x) (__FlashStringHelper*)(x)
  
  #include <Wire.h>
  #include <Arduino.h>
  #include <U8x8lib.h>
  
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
  unsigned long lastScreenUpdate = 0; 
  bool doScreenUpdate = false;
  bool displayStats = false;
  int currentScreen = -1;

  const char clearLine[] PROGMEM = { "                " };
  const char space[] PROGMEM = { " " };
#endif

#ifdef SD_CARD
  #include <SPI.h>
  #include <SD.h>
  int pinCS = 8;
  unsigned long lastSdWrite = 0;
  File myFile;
#endif

#ifdef BME280
  ss//https://github.com/finitespace/BME280
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  Adafruit_BME280 bme;
  float PressureAltitude, MaxPressureAltitude = 0;
  float gndPressure = 0;
#endif

#ifdef TEMP1
  #include <SimpleKalmanFilter.h>  
  SimpleKalmanFilter temp1Filter(0.03, 0.003, 0.03);
  int T1, T1Max = 0;
#endif

#ifdef TEMP2
  #include <SimpleKalmanFilter.h>  
  SimpleKalmanFilter temp2Filter(0.03, 0.003, 0.03);
  int T2, T2Max = 0;
#endif

bool buttonPressed = false;

void setup()
{
#ifdef SD_CARD
  initSDCard();
#endif

#ifdef GPS_BAUD
  initGPS();
#endif

#ifdef OLED
  initOled();
  displayHeader();
#endif

#ifdef BME280
  initBme();
#endif

  initButton();
}

#ifdef SD_CARD
void initSDCard()
{
  pinMode(pinCS, OUTPUT);
  SD.begin(pinCS);
}
#endif

#ifdef GPS_BAUD
void initGPS()
{
  gpsPort.begin(GPS_BAUD);
}
#endif

#ifdef OLED
void initOled()
{  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}
#endif

void initButton()
{
  pinMode(BUTTON_INPUT,INPUT);
  digitalWrite(BUTTON_INPUT,HIGH);
}

#ifdef BME280
void initBme()
{
  bme.begin();
  //at init read pressure at gnd level
  gndPressure = 1013.25;
}
#endif

void loop()
{  
  unsigned long now = millis();
#ifdef OLED
  if ( now - lastScreenUpdate < LCD_REFRESH ) {
    doScreenUpdate = false;
  } else {
    doScreenUpdate = true;
    lastScreenUpdate = now;
  }
#endif
  
#ifdef GPS_BAUD
  readGPS();

  if(numSV >= GPS_MIN_SAT && zeroingCounter > 0 && now - lastZeroingUpdate >= 1000)
  {
    zeroingCounter = zeroingCounter - 1;  
    lastZeroingUpdate = now;
  }
#endif

  readButtonPressed();
  performReadouts();

#ifdef OLED
  if(doScreenUpdate)
  {
    updateScreen();
    lastScreenUpdate = millis();
  }
#endif

#ifdef SD_CARD
  now = millis();
  if(now - lastSdWrite >= SD_LOG_TIME)
  {
    logToSD();
    lastSdWrite = now;
  }
#endif
}

#ifdef OLED
void updateScreen()
{
  if (numSV < GPS_MIN_SAT)
  {
    if(currentScreen != 0)
    {
      printWaitingLayout();
      currentScreen = 0;
    }
    u8x8.setCursor(5,3);
    if(numSV < 10)
      u8x8.print(FS(space));
    u8x8.print(numSV);
  }
  else
  {
    if(displayStats)
    {
      displayStatistics();
    }
    else
    {
      displayCurrentReadouts();
    }
  }

  if(buttonPressed)
    displayStats = not displayStats;
}

void printWaitingLayout()
{
    clearLines(2); 
    u8x8.setCursor(0, 2);
    u8x8.print(F("Waiting for sats"));
    u8x8.setCursor(0, 3);
    u8x8.print(F("Sats:"));
}

void displayHeader()
{
  u8x8.setCursor(0,0);
  u8x8.print(FS(clearLine));
  u8x8.print(F("GPS Logger v2"));  
}

void displayCurrentReadouts()
{
  if(currentScreen != 1)
  {
    clearLines(2);
    displayCurrentReadoutsLayout();
    currentScreen = 1;
  }
    
#ifdef GPS_BAUD
  u8x8.setCursor(0, 2);
  u8x8.print(F("Sats:"));
  u8x8.print(numSV);

  u8x8.setCursor(0, 3);
  u8x8.print(F("Speed:"));
  u8x8.print(Speed);
  u8x8.print(F("km/h"));

  u8x8.setCursor(0, 4);
  u8x8.print(F("GPS Alt:"));
  u8x8.print(Height);
  u8x8.print(F("m"));
#endif
  
  displayCurrentTemp();

#ifdef BME280
  u8x8.setCursor(0,6);
  u8x8.print(F("Max Pres Alt:"));
  u8x8.print(MaxPressureAltitude);
#endif
}

void displayCurrentReadoutsLayout()
{
      
#ifdef GPS_BAUD
  u8x8.setCursor(0, 2);
  u8x8.print(F("Sats:"));
  
  u8x8.setCursor(0, 3);
  u8x8.print(F("Speed:"));
  u8x8.setCursor(9, 3);
  u8x8.print(F("km/h"));

  u8x8.setCursor(0, 4);
  u8x8.print(F("GPS Alt:"));
  u8x8.setCursor(11, 4);
  u8x8.print(F("m"));
#endif

  //do fixed layout here
  displayCurrentTemp();

#ifdef BME280
  u8x8.setCursor(0,6);
  u8x8.print(F("Max Pres Alt:"));
#endif
}

void displayStatistics()
{ 
  clearLines(2);    
  
  u8x8.setCursor(0, 2); 
  u8x8.print(F("Sats:"));
  u8x8.print(numSV);

  
  u8x8.setCursor(0, 3);
  u8x8.print(F("Max spd:"));
  u8x8.print(MaxSpeed);
  u8x8.print(F("km/h"));
  
  u8x8.setCursor(0, 4);
  
  if(zeroingCounter > 0)
  {
    u8x8.print(F("Zeroing alt  "));
    if(zeroingCounter > 9)
    {
      u8x8.print(zeroingCounter);
    }
    else
    {
      u8x8.print(F(" ")); 
      u8x8.print(zeroingCounter);
    }
  }
  else
  {      
    u8x8.print(F("Max alt:"));
    u8x8.print(MaxHeight - ZeroHeight);
    u8x8.println("m");
  }

  displayMaxTemp();

#ifdef BME280
  u8x8.setCursor(0,7);
  u8x8.print(F("Pres Alt:"));
  u8x8.print(PressureAltitude);
#endif
}

void displayCurrentTemp()
{
  u8x8.setCursor(0, 5);  
#ifdef TEMP1
  u8x8.print(F("TB:"));
  u8x8.print(T1);
  u8x8.print(F("C "));
#endif

#ifdef TEMP2
  u8x8.print(F("TG:"));
  u8x8.print(T2); 
  u8x8.print(F("C"));
#endif
}

void displayMaxTemp()
{  
#ifdef TEMP1
  u8x8.setCursor(0, 5);
  u8x8.print(F("TB max:"));
  u8x8.print(T1Max);  
  u8x8.print(F("C"));
#endif

#ifdef TEMP2
  u8x8.setCursor(0, 6);
  u8x8.print(F("TG max:"));
  u8x8.print(T2Max); 
  u8x8.print(F("C"));
#endif
}

void clearLines(int startingLine)
{
    for(startingLine; startingLine < 8; startingLine++)
    {
      u8x8.setCursor(0,startingLine);
      u8x8.print(FS(clearLine));
    }
}
#endif

#ifdef SD_CARD
void logToSD()
{
  //filename from gps https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Data%20Model.md#usage
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) 
  {
//timestamp
#ifdef GPS_BAUD
    myFile.print(Height);
    myFile.print(F(";"));
    myFile.print(Speed);
    myFile.print(F(";"));
#endif

#ifdef TEMP1
    myFile.print(T1);
    myFile.print(F(";"));
#endif

#ifdef TEMP2
    myFile.print(T2);
    myFile.print(F(";"));
#endif

#ifdef  BME280
    myFile.print(PressureAltitude);
    myFile.println();
#endif

    myFile.println();
    myFile.close();
  }
}
#endif

#ifdef GPS_BAUD
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
    else
    {
      Speed = 0; 
    }
    
    if(fix.valid.altitude)
    {
      Height = (int)gpsAltitudeFilter.updateEstimate(fix.alt.whole);
    }
    else
    {
      Height = 0;
    }
  }
}
#endif

void readButtonPressed()
{
  buttonPressed = (digitalRead(BUTTON_INPUT) == 0);
}

void performReadouts()
{
#ifdef TEMP1
  T1 = temp1Filter.updateEstimate(calculateRawTemp(TEMP1));
  if(T1 > T1Max)
    T1Max = T1;
#endif

#ifdef TEMP2
  T2 = temp2Filter.updateEstimate(calculateRawTemp(TEMP2));
  if(T2 > T2Max)
    T2Max = T2;
#endif

#ifdef GPS_BAUD
  if (numSV >= GPS_MIN_SAT)
  {
    if (Speed > MaxSpeed)
    {
      MaxSpeed = Speed;
    }
    
    if(Height > 0 && zeroingCounter > 0)
    {
      ZeroHeight = Height;
    }
    
    if (Height > MaxHeight && zeroingCounter <= 0)
    {
      MaxHeight = Height;
    }
  }
#endif

#ifdef BME280
  PressureAltitude = bme.readAltitude(gndPressure);
  if(PressureAltitude > MaxPressureAltitude)
    MaxPressureAltitude = PressureAltitude;
#endif
}

#if defined(TEMP1) || defined(TEMP2)
float calculateRawTemp(int port)
{
    int readVal = analogRead(port);
    float volt = readVal * 5.0 / 1024.0;
    return (volt - 0.5) * 100;
}
#endif
