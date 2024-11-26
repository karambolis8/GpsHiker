#include "Config.h"

#include "Utils.h"
#include "Temp.h"
#include "BME280.h"
#include "TempOled.h"
#include "BME280Oled.h"

int T1, T1Max = 0;
unsigned long lastTempReadout = 0;
int PressureAltitude, MaxPressureAltitude = 0;
unsigned long lastPerformedReadouts = 0;

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

#include <Wire.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
unsigned long lastScreenUpdate = 0; 
bool doScreenUpdate = false;
int previousScreen = -1;
int currentScreen = 1;

bool blink = true;



void setup()
{
  initOled();
  displayHeader();

#ifdef GPS_BAUD
  u8x8.setCursor(0,2);
  u8x8.print(F("Initializing GPS"));
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initGPS();
#endif

  u8x8.setCursor(0,3);
  u8x8.print(F("Initializing BME")); 
  delay(OLED_SENSOR_CALIBRATION_DELAY);
  initBme();

  initTempSensor();

#ifdef BUTTON_INPUT
  initButton();
#endif
}

void initOled()
{  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

#ifdef BUTTON_INPUT
//implement debounce https://projecthub.arduino.cc/ronbentley1/button-switch-using-an-external-interrupt-16d57f
void initButton()
{
  pinMode(BUTTON_INPUT,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), buttonPressed, LOW);
}

void buttonPressed()
{   
  currentScreen = ((currentScreen + 1) % 3 ) +1;
}
#endif


void loop()
{
  unsigned long now = millis();
  
  if ( now - lastScreenUpdate < OLED_REFRESH ) {
    doScreenUpdate = false;
  } else {
    doScreenUpdate = true;
    lastScreenUpdate = now;
  }
  
#ifdef GPS_BAUD
    readGPS();
#endif

  performReadouts();
  lastPerformedReadouts = millis();  

  if(doScreenUpdate)
  {
    updateScreen();
    lastScreenUpdate = millis();
  }
}

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

  displayCurrentTemp(u8x8, T1);
  displayCurrentBmeAlt(u8x8, PressureAltitude);
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

  displayCurrentTempLayout(u8x8);
  displayCurrentBmeAltLayout(u8x8);
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

  displayMaxTemp(u8x8, T1Max);
  displayMaxBmeAlt(u8x8, MaxPressureAltitude);
}

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

  displayMaxTempLayout(u8x8);
  displayMaxBmeAltLayout(u8x8);
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
  calculateGps();
  calculateTemp();
  calculatePressureAlt();
}

void calculatePressureAlt()
{
  int presureAltReadout = calculateBmeAlt(); 
  
  if(presureAltReadout > MaxPressureAltitude)
    MaxPressureAltitude = presureAltReadout;
}

void calculateTemp()
{
  if(millis() - lastTempReadout >= ANALOG_READ_DELAY)
  {
    T1 = calculateRawTemp();
    if(T1 > T1Max)
      T1Max = T1;

    lastTempReadout = millis();
  }
}
