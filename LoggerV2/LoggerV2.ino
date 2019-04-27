
#include "Config.h"

const float VccReference = 4.55;

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
  bool refreshDisplay = true;
  int currentScreen = 2;

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
  int currentOffsetSize = 10;
  float MaxCurrent, amps, currentOffset = 0;
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

#ifdef GYRO
  #include "Wire.h"
  #include "MPU6050.h"
  MPU6050 mpu;
  float gx, gy, gz, maxGx, maxGy, maxGz = 0;
  int offsetCalibrationSize = 10;
  float offsetX, offsetY, offsetZ = 0;
#endif

bool buttonPressed = false;
unsigned long buttonDebounce = 0;

void setup()
{
  Serial.begin(9600);
  
#ifdef OLED
  initOled();
  displayHeader();
#endif

#ifdef SD_CARD
#ifdef OLED
  u8x8.setCursor(0,1);
  u8x8.print(F("Initializing SD"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  initSDCard();
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

#ifdef AIRSPEED
#ifdef OLED
  u8x8.setCursor(0,4);
  u8x8.print(F("Initializing AIR"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  setupAirSpeed(airSpeedPin);
#endif

#ifdef GYRO
#ifdef OLED
  u8x8.setCursor(0,5);
  u8x8.print(F("Initializing MPU"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  initMpu();
#endif

#ifdef CURRENT_SENSOR
#ifdef OLED
  u8x8.setCursor(0,6);
  u8x8.print(F("Initializing ACS"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
#endif
  initCurrent();
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

#ifdef GYRO
void initMpu()
{
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G);
  delay(100);
  
  for(int i = 0; i < 10; i++)
  {
    Vector accel = mpu.readNormalizeAccel();;
    offsetX += accel.XAxis;
    offsetY += accel.YAxis;
    offsetZ += accel.ZAxis;
  }

  offsetX /= 10.0;
  offsetY /= 10.0;
  offsetZ /= 10.0;
}

void calculateMpu()
{
  Vector accel = mpu.readNormalizeAccel();
  
  gx = abs(accel.XAxis - offsetX);
  if(gx > maxGx)
    maxGx = gx;
    
  gy = abs(accel.YAxis - offsetY);
  if(gy > maxGy)
    maxGy = gy;
    
  gz = abs(accel.ZAxis - offsetZ);
  if(gz > maxGz)
    maxGz = gz;
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
    if(refreshDisplay)
    {
      printWaitingLayout();
      refreshDisplay = false;
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
    else if(currentScreen == 3)
    {
      displayAcceleration();
    }
#ifdef GPS_BAUD
  }
#endif

  if(buttonPressed)
  {
    currentScreen = ((currentScreen + 1) % 3 ) +1;
    refreshDisplay = true;
  }

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
  if(refreshDisplay)
  {
    clearLines(1);
    displayCurrentReadoutsLayout();
    refreshDisplay = false;
  }
    
#ifdef GPS_BAUD
  u8x8.setCursor(5, 1);
  if(numSV < 10)
    u8x8.print(FS(space));
  u8x8.print(numSV);

  u8x8.setCursor(8, 2);
  if(Speed <= 99)
    u8x8.print(FS(space));
  if(Speed <= 9)
    u8x8.print(FS(space));
  u8x8.print(Speed);

  u8x8.setCursor(8, 3);
  if(Height <= 999)
    u8x8.print(FS(space));
  if(Height <= 99)
    u8x8.print(FS(space));
  if(Height <= 9)
    u8x8.print(FS(space));
  u8x8.print(Height);
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

#ifdef CURRENT_SENSOR
  u8x8.setCursor(4,6);
  if(amps <= 10)
      u8x8.print(FS(space));
  u8x8.print(String(amps, 1));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(8,4);
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
  u8x8.print(F("GPS Spd:"));
  u8x8.setCursor(11, 2);
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
  u8x8.setCursor(12,5);
  u8x8.print(F("m"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Amp:"));
  u8x8.setCursor(8,6);
  u8x8.print(F("A"));
#endif

#ifdef AIRSPEED
  u8x8.setCursor(0,4);
  u8x8.print(F("Air Spd:"));
  u8x8.setCursor(12,4);
  u8x8.print(F("km/h"));
#endif
}

void displayStatistics()
{   
  if(refreshDisplay)
  {
    clearLines(1);
    displayStatisticsLayout();
    refreshDisplay = false;
  }

#ifdef GPS_BAUD
 if(currentScreen == 2 && zeroingCounter == 0)
 {
    clearLines(1);
    displayStatisticsLayout();  
 }

  u8x8.setCursor(5, 1); 
  if(numSV < 10)
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
    u8x8.setCursor(12, 3);
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
  u8x8.setCursor(8,5);  
    if(MaxPressureAltitude <= 999)
    u8x8.print(FS(space));
  if(MaxPressureAltitude <= 99)
    u8x8.print(FS(space));
  if(MaxPressureAltitude <= 9)
    u8x8.print(FS(space));
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
  u8x8.print(F("Max GPS:"));
  u8x8.setCursor(11, 2);
  u8x8.print(F("km/h"));
  
  u8x8.setCursor(0, 3);  
  if(zeroingCounter > 0)
  {
    u8x8.print(F("Zeroing alt"));
  }
  else
  {
    Serial.println(zeroingCounter);
    if(zeroingCounter == 0)
    {
      u8x8.print(FS(clearLine));
      zeroingCounter -= 1; 
    }
    
    u8x8.print(F("Max GPS:"));
    u8x8.setCursor(12, 3);
    u8x8.print(F("m"));
  }
#endif

#ifdef TEMP
  displayMaxTempLayout();
#endif

#ifdef BME280
  u8x8.setCursor(0,5);
  u8x8.print(F("Max Bar:"));
    u8x8.setCursor(12, 5);
    u8x8.print(F("m"));
#endif

#ifdef CURRENT_SENSOR
  u8x8.setCursor(0,6);
  u8x8.print(F("Max Amp:"));
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

#ifdef GYRO
void displayAcceleration()
{
  if(refreshDisplay)
  {
    clearLines(1);
    displayAccelerationLayout();
    refreshDisplay = false;
  }
  
  u8x8.setCursor(6, 1);
  if(gx < 100)
    u8x8.print(FS(space));
  if(gx < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(gx, 1));
  
  u8x8.setCursor(6, 2);
  if(gy < 100)
    u8x8.print(FS(space));
  if(gy < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(gy, 1));
  
  u8x8.setCursor(6, 3);
  if(gz < 100)
    u8x8.print(FS(space));
  if(gz < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(gz, 1));
  
  u8x8.setCursor(6, 4);
  if(maxGx < 100)
    u8x8.print(FS(space));
  if(maxGx < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(maxGx, 1));
  
  u8x8.setCursor(6, 5);
  if(maxGy < 100)
    u8x8.print(FS(space));
  if(maxGy < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(maxGy, 1));
  
  u8x8.setCursor(6, 6);
  if(maxGz < 100)
    u8x8.print(FS(space));
  if(maxGz < 10)
    u8x8.print(FS(space));    
  u8x8.print(String(maxGz, 1));
}

void displayAccelerationLayout()
{
  u8x8.setCursor(0, 1);
  u8x8.print(F("Acc X:"));
  u8x8.setCursor(11, 1);
  u8x8.print(F("m/s2"));
  u8x8.setCursor(0, 2);
  u8x8.print(F("Acc Y:"));
  u8x8.setCursor(11, 2);
  u8x8.print(F("m/s2"));
  u8x8.setCursor(0, 3);
  u8x8.print(F("Acc Z:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("m/s2"));
  u8x8.setCursor(0, 4);
  u8x8.print(F("Max X:"));
  u8x8.setCursor(11, 4);
  u8x8.print(F("m/s2"));
  u8x8.setCursor(0, 5);
  u8x8.print(F("Max Y:"));
  u8x8.setCursor(11, 5);
  u8x8.print(F("m/s2"));
  u8x8.setCursor(0, 6);
  u8x8.print(F("Max Z:"));
  u8x8.setCursor(11, 6);
  u8x8.print(F("m/s2"));
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
    
    if(fix.valid.altitude)
    {
      Height = (int)gpsAltitudeFilter.updateEstimate(fix.alt.whole);
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
    
    if (Height-ZeroHeight > MaxHeight && zeroingCounter <= 0)
    {
      MaxHeight = Height - ZeroHeight;
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

#ifdef GYRO
  calculateMpu();
#endif
}

#ifdef CURRENT_SENSOR

void initCurrent()
{
  int sum = 0;
  
  for(int i = 0 ; i < currentOffsetSize; i++)
  {
    sum += analogRead(currentPin);
  }

  sum /= currentOffsetSize;
  currentOffset = 512 - sum;
}

void calculateCurrent()
{
  float currentSensorVoltage = ((analogRead(currentPin) - currentOffset) / 1024.0) * VccReference;

  Serial.println(currentSensorVoltage);

  float sensorAmps = (currentSensorVoltage - (VccReference/2)) / (mvPerAmp * 0.001);

  Serial.println(analogRead(currentPin));
  
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
