
#include "Config.h"

const float VccReference = 4.550;

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
  
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter gpsAltitudeFilter(0.03, 0.003, 0.03); 

  NMEAGPS  gps;
  gps_fix  fix;
  int zeroingCounter = GPS_ZEROING_TIME;
  unsigned long lastZeroingUpdate = 0;
  long Speed = 0;
  long MaxSpeed = 0;
  int Height = 0;
  int MaxHeight, ZeroHeight = 0;
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
  bool displayStats = false;
  int currentScreen = -1;

  const char clearLine[] PROGMEM = { "                " };
  const char space[] PROGMEM = { " " };

  bool blink = true;
#endif

#ifdef SD_CARD
  #include <SPI.h>
  #include <SD.h>
  
  int pinCS = 8;
  unsigned long lastSdWrite = 0;
  File myFile;
#endif

#ifdef BME280
  //#include <Arduino.h>
  #include <Wire.h>  
  #include <BMx280MI.h>
  
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter pressAltFilter(0.03, 0.003, 0.03);

  BMx280I2C bmx280(0x76);
  float PressureAltitude, MaxPressureAltitude = 0;
  float gndPressure = 0;
#endif

#ifdef TEMP
  #include <SimpleKalmanFilter.h>  
  SimpleKalmanFilter temp1Filter(0.03, 0.003, 0.03);
  
  int T1, T1Max = 0;
  int tempPin = 0;
#endif

#ifdef CURRENT_SENSOR
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter currentFilter(0.03, 0.003, 0.03); 
  
#ifdef ACS758_50B
  int mvPerAmp = 40;
#endif
#ifdef ACS712_20B
  int mvPerAmp = 100;
#endif

  int currentPin = 6;
  double currentSensorVoltage = 0;
  double MaxCurrent, amps = 0;
#endif

#ifdef AIRSPEED
  #include <SimpleKalmanFilter.h>
  SimpleKalmanFilter airSpeedFilter(0.03, 0.003, 0.03); 
  
  float rho = 1.204; // density of air   
  int zeroSpan = 2;
  int offset = 0;
  int offsetSize = 10;
  int airSpeedPin = 7;
  int airSpeed, maxAirSpeed = 0;
#endif

bool buttonPressed = false;
unsigned long buttonDebounce = 0;

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
  bmx280.begin();
  bmx280.resetToDefaults();
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
}

void calculatePressAlt()
{
  bmx280.measure();
  while (!bmx280.hasValue());  
  
  float bmeSensorRaw = bmx280.getPressure();
  if(bmeSensorRaw < 0)
	  bmeSensorRaw = 0;
   
  PressureAltitude = pressAltFilter.updateEstimate(bmeSensorRaw);
  
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

#ifdef TEMP
  displayCurrentTemp();
#endif

#ifdef BME280
  u8x8.setCursor(9,5);
  u8x8.print(PressureAltitude);
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(5,6);
  if(amps <= 10)
      u8x8.print(FS(space));
  u8x8.print(String(amps, 1));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(7,4);
  if(airSpeed >= 0)
      u8x8.print(FS(space));
  if(abs(airSpeed) <= 99)
      u8x8.print(FS(space));
  if(abs(airSpeed) <= 9)
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

#ifdef TEMP
  displayCurrentTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Bar Alt:"));
  u8x8.setCursor(11,5);
  u8x8.print(F("m"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Amps:"));
  u8x8.setCursor(9,6);
  u8x8.print(F("A"));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(0,4);
  u8x8.print(F("AirSpd:"));
  u8x8.setCursor(11,4);
  u8x8.print(F("km/h"));
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

#ifdef TEMP
  displayMaxTemp();
#endif

#ifdef BME280
  u8x8.setCursor(13,5);
  u8x8.print(MaxPressureAltitude);
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(9,6);
  if(MaxCurrent <= 10)
      u8x8.print(FS(space));
  u8x8.print(String(MaxCurrent, 1));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(8,4);
  if(maxAirSpeed >= 0)
      u8x8.print(FS(space));
  if(abs(maxAirSpeed) <= 99)
      u8x8.print(FS(space));
  if(abs(maxAirSpeed) <= 9)
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

#ifdef TEMP
  displayMaxTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Max bar Alt:"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Max Amps:"));
  u8x8.setCursor(13,6);
  u8x8.print(F("A"));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(0,4);
  u8x8.print(F("Max Air:"));
  u8x8.setCursor(12,4);
  u8x8.print(F("km/h"));
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
  u8x8.print(F("Temp:"));
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
  u8x8.print(F("Max Temp:"));
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

#ifdef SD_CARD
void initSDCard()
{
  pinMode(pinCS, OUTPUT);
  SD.begin(pinCS);
}

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

#ifdef TEMP
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

void calculateGps()
{
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
}
#endif

void readButtonPressed()
{
  buttonPressed = (digitalRead(BUTTON_INPUT) == 0);
  
//  unsigned long now = millis();
//  if((digitalRead(BUTTON_INPUT) == 0) && now - buttonDebounce >= BUTTON_DELAY)
//  {
//	buttonPressed = true;
//	buttonDebounce = now;
//  }
//  else
//  {
//	  buttonPressed = false;
//  }	  
}

void performReadouts()
{
#ifdef TEMP
  calculateTemp();
#endif

#ifdef GPS_BAUD
  calculateGps();
#endif

#ifdef BME280
  calculatePressAlt();
#endif

#ifdef CURRENT_SENSOR
  calculateCurrent();
#endif

#ifdef AIRSPEED
  calculateAirSpeed();
#endif
}

#ifdef CURRENT_SENSOR
void calculateCurrent()
{
  currentSensorVoltage = (analogRead(currentPin) / 1024.0) * VccReference;

  float sensorAmps = (currentSensorVoltage - VccReference/2) / mvPerAmp;
  
  if(sensorAmps < 0)
    sensorAmps = 0.0;

  amps = currentFilter.updateEstimate(sensorAmps);

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

void calculateAirSpeed()
{	  
  airSpeed = airSpeedFilter.updateEstimate(calculateRawAirSpeed(airSpeedPin));
  if(airSpeed > maxAirSpeed)
    maxAirSpeed = airSpeed;
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
      return (-sqrt((-10000.0*((rawSensor/1024.0)-0.5))/rho)) * 36.0 / 10.0;
    } 
    else
    {
      return sqrt((10000.0*((rawSensor/1024.0)-0.5))/rho)  * 36.0 / 10.0;
    }
  }
}
#endif

#ifdef TEMP
void calculateTemp()
{
  T1 = temp1Filter.updateEstimate(calculateRawTemp(tempPin));
  if(T1 > T1Max)
    T1Max = T1;
}

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
    float volt = readVal* VccReference / 1024.0 ;
    return (volt - 0.5) * 100.0;
}
#endif
#endif
