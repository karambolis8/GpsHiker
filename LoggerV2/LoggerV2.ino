//https://electronics.stackexchange.com/questions/25278/how-to-connect-multiple-i2c-interface-devices-into-a-single-pin-a4-sda-and-a5

#include "Config.h"

const int VccReference = 5000;

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
  int tempPin = 0;
#endif

#ifdef CURRENT_SENSOR
#ifdef ACS758_50B
  int mvPerAmp = 40;
#endif
#ifdef ACS712_20B
  int mvPerAmp = 100;
#endif
  int currentPin = 2;
  double currentSensorVoltage = 0;
  double MaxCurrent, amps = 0;
#endif

#ifdef AIRSPEED
  #include <SimpleKalmanFilter.h>
  
  float rho = 1.204; // density of air   
  int zeroSpan = 2;
  int offset = 0;
  int offsetSize = 10;
  int airSpeedPin = 7;
  int airSpeed, maxAirSpeed = 0;
  SimpleKalmanFilter airSpeedFilter(0.03, 0.003, 0.03); 
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

#ifdef AIRSPEED
  setupAirSpeed(airSpeedPin);
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
#ifdef GPS_BAUD
  if (numSV < GPS_MIN_SAT)
  {
    if(currentScreen != 0)
    {
      printWaitingLayout();
      currentScreen = 0;
    }
    u8x8.setCursor(5,2);
    if(numSV < 10)
      u8x8.print(FS(space));
    u8x8.print(numSV);
  }
  else
  {
#endif
    if(displayStats)
    {
      displayStatistics();
    }
    else
    {
      displayCurrentReadouts();
    }
#ifdef GPS_BAUD
  }
#endif

  if(buttonPressed)
    displayStats = not displayStats;
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
  if(currentScreen != 1)
  {
    clearLines(1);
    displayCurrentReadoutsLayout();
    currentScreen = 1;
  }
    
#ifdef GPS_BAUD
  u8x8.setCursor(0, 1);
  u8x8.print(F("Sats:"));
  u8x8.print(numSV);

  u8x8.setCursor(0, 2);
  u8x8.print(F("Speed:"));
  u8x8.print(Speed);
  u8x8.print(F("km/h"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Alt:"));
  u8x8.print(Height);
  u8x8.print(F("m"));
#endif

#ifdef TEMP1
  displayCurrentTemp();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Press Alt:"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(5,6);
  if(amps <= 10)
      u8x8.print(FS(space));
  u8x8.print(String(amps, 1));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(9,7);
  if(airSpeed <= 99)
      u8x8.print(FS(space));
  if(airSpeed <= 9)
      u8x8.print(FS(space));
  u8x8.print(airSpeed);
#endif
}

void displayCurrentReadoutsLayout()
{
      
#ifdef GPS_BAUD
  u8x8.setCursor(0, 1);
  u8x8.print(F("Sats:"));
  
  u8x8.setCursor(0, 2);
  u8x8.print(F("Speed:"));
  u8x8.setCursor(9, 2);
  u8x8.print(F("km/h"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Alt:"));
  u8x8.setCursor(12, 3);
  u8x8.print(F("m"));
#endif

#ifdef TEMP1
  displayCurrentTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Press Alt:"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Amps:"));
  u8x8.setCursor(9,6);
  u8x8.print(F("A"));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(0,7);
  u8x8.print(F("AirSpeed:"));
  u8x8.setCursor(12,7);
  u8x8.print(F("m/s"));
#endif
}

void displayStatistics()
{   
  if(currentScreen != 2)
  {
    clearLines(1);
    displayStatisticsLayout();
    currentScreen = 2;
  }  

#ifdef GPS_BAUD
  u8x8.setCursor(5, 1); 
  if(numSV < 9)
    u8x8.print(FS(space));
  u8x8.print(numSV);
  
  u8x8.setCursor(8, 2);
  if(MaxSpeed <= 99)
    u8x8.print(FS(space));
  if(MaxSpeed <= 9)
    u8x8.print(FS(space));
  u8x8.print(MaxSpeed);
    
  if(zeroingCounter > 0)
  {
    u8x8.setCursor(11, 3);
    if(zeroingCounter <= 9)
      u8x8.print(FS(space));
    u8x8.print(zeroingCounter);
  }
  else
  {
    u8x8.setCursor(8, 3);
    if(MaxHeight <= 999)
      u8x8.print(FS(space));
    if(MaxHeight <= 99)
      u8x8.print(FS(space));
    if(MaxHeight <= 9)
      u8x8.print(FS(space));
    u8x8.print(MaxHeight);
  }
#endif

#ifdef TEMP1
  displayMaxTemp();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Max press Alt:"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(9,6);
  if(MaxCurrent <= 10)
      u8x8.print(FS(space));
  u8x8.print(String(MaxCurrent, 1));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(8,7);
  if(maxAirSpeed <= 99)
      u8x8.print(FS(space));
  if(maxAirSpeed <= 9)
      u8x8.print(FS(space));
  u8x8.print(maxAirSpeed);
#endif
}

#ifdef OLED
void displayStatisticsLayout()
{
  #ifdef GPS_BAUD
  u8x8.setCursor(0, 1); 
  u8x8.print(F("Sats:"));
  
  u8x8.setCursor(0, 2);
  u8x8.print(F("Max spd:"));
  u8x8.setCursor(11, 2);
  u8x8.print(F("km/h"));
  
  u8x8.setCursor(0, 3);  
  if(zeroingCounter > 0)
  {
    u8x8.print(F("Zeroing alt"));
  }
  else
  {      
    u8x8.print(F("Max alt:"));
    u8x8.setCursor(13, 3);
    u8x8.println("m");
  }
#endif

#ifdef TEMP1
  displayMaxTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Max press Alt:"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Max Amps:"));
  u8x8.setCursor(13,6);
  u8x8.print(F("A"));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(0,7);
  u8x8.print(F("Max Air:"));
  u8x8.setCursor(11,7);
  u8x8.print(F("m/s"));
#endif
}
#endif

#ifdef TEMP1
void displayCurrentTemp()
{
  u8x8.setCursor(5, 4);  
  if(T1 > 0)  
    u8x8.print(FS(space));
  if(abs(T1) <= 99)
    u8x8.print(FS(space));
  if(abs(T1) <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1);
}

void displayCurrentTempLayout()
{
  u8x8.setCursor(0, 4);
  u8x8.print(F("Temp:"));
  u8x8.setCursor(9, 4);  
  u8x8.print(F("C"));
}

void displayMaxTemp()
{
  u8x8.setCursor(9, 4);
  if(T1Max > 0)  
    u8x8.print(FS(space));
  if(abs(T1Max) <= 99)
    u8x8.print(FS(space));
  if(abs(T1Max) <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1Max);
}

void displayMaxTempLayout()
{
  u8x8.setCursor(0, 4);
  u8x8.print(F("Max Temp:"));
  u8x8.setCursor(13, 4); 
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
  T1 = temp1Filter.updateEstimate(calculateRawTemp(tempPin));
  if(T1 > T1Max)
    T1Max = T1;
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

#ifdef CURRENT_SENSOR
  calculateCurrent();
#endif

#ifdef AIRSPEED
  airSpeed = airSpeedFilter.updateEstimate(calculateRawAirSpeed(airSpeedPin));
  if(airSpeed > maxAirSpeed)
    maxAirSpeed = airSpeed;
#endif
}

#ifdef CURRENT_SENSOR
void calculateCurrent()
{
  currentSensorVoltage = (analogRead(currentPin) / 1023.0)*VccReference;
  //add logic with -512 +- zeroSpan
  amps = (currentSensorVoltage-VccReference/2)/mvPerAmp;
  if(amps < 0)
    amps = 0;
  if(MaxCurrent < amps)
    MaxCurrent = amps;
}
#endif

#ifdef AIRSPEED
  void setupAirSpeed(int port)
  {    
    for (int ii=0;ii<offsetSize;ii++){
      offset += analogRead(port)-(1023/2);
    }
    offset /= offsetSize;
  }

  float calculateRawAirSpeed(int port)
  {
    int rawSensor = analogRead(port) - offset;
    if(rawSensor > 512 - zeroSpan && rawSensor < 512 + zeroSpan) 
	{
	  return 0.0;	
    }
    else
    {
      if (rawSensor < 512)
      {
        return -sqrt((-10000.0*((rawSensor/1023.0)-VccReference/20000))/rho);
      } 
      else
      {
        return sqrt((10000.0*((rawSensor/1023.0)-VccReference/20000))/rho);
      }
    }
  }
#endif

#ifdef LM35
int calculateRawTemp(int port)
{
    int readVal = analogRead(port);
    float volt = readVal * VccReference / 1024.0 / 1000;
    return volt * 100;
}
#endif

#ifdef TMP36
int calculateRawTemp(int port)
{
    int readVal = analogRead(port);
    float volt = readVal * VccReference / 1024.0 / 1000;
    return (volt - VccReference/20000) * 100;
}
#endif