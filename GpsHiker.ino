#include "Config.h"

#include "Utils.h"
#include "Temp.h"
#include "BME280.h"
#include "TempOled.h"
#include "BME280Oled.h"

int T1, T1Max = 0;
unsigned long lastTempReadout = 0;
int PressureAltitude, MaxPressureAltitude = 0;
int Humidity, MaxHumidity, MinHumidity = 0;
unsigned long lastPerformedReadouts = 0;

#ifdef GPS_BAUD
  #include "NMEAGPS.h"
  #include "GPSport.h"
  #include "NeoTime.h"
  
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
  int year;
  int month;
  int day;
  int hour;
  int minutes;
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
  initButton();
}

void initOled()
{  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void initButton()
{
  pinMode(BUTTON_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), buttonPressed, RISING);
}

void buttonPressed()
{
    if (interrupt_process_status == !BUTTON_TRIGGERED) 
    {
      if (digitalRead(BUTTON_INPUT) == HIGH) 
      {
        interrupt_process_status  = BUTTON_TRIGGERED;
      }
    }
}

bool readButton()
{
  int button_reading;
  
  static bool     switching_pending = false;
  static  long int elapse_timer;
  if (interrupt_process_status == BUTTON_TRIGGERED) 
  {    
    button_reading = digitalRead(BUTTON_INPUT);
    if (button_reading == HIGH)  
    {
      switching_pending = true;
      elapse_timer  = millis();
    }
    if (switching_pending  && button_reading == LOW) 
    {
      if (millis() - elapse_timer >= DEBOUNCE) 
      {
        switching_pending  = false;
        interrupt_process_status  = !BUTTON_TRIGGERED;
        return BUTTON_SWITCHED;
      }
    }
  }
  return !BUTTON_SWITCHED;
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
    
    // u8x8.setCursor(15,1);
    // u8x8.print(currentScreen);
  }
  else
  {
#endif
    if(currentScreen == 2)
    {
      displayStatistics();
    }
    else if(currentScreen == 1)
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

  // u8x8.setCursor(15,1);
  // u8x8.print(currentScreen);

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
  
#endif

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
#ifdef GPS_BAUD
  u8x8.setCursor(0, 1);
  u8x8.print(F("Sats:"));

  u8x8.setCursor(0, 2);
  u8x8.print(F("Date:"));

  u8x8.setCursor(0, 6);
  u8x8.print(F("Time:"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("GPS Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));
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
#endif

  displayMaxTemp(u8x8, T1Max);
  displayBmeAlt(u8x8, MaxPressureAltitude);
  displayBmeHumidity(u8x8, MaxHumidity);
}

void displayStatisticsLayout()
{
  #ifdef GPS_BAUD
  u8x8.setCursor(0, 1); 
  u8x8.print(F("Sats:"));  

  u8x8.setCursor(0, 2);
  u8x8.print(F("Date:"));

  u8x8.setCursor(0, 3);
  u8x8.print(F("Max Spd:"));
  u8x8.setCursor(11, 3);
  u8x8.print(F("km/h"));
#endif

  displayMaxTempLayout(u8x8);
  displayMaxBmeAltLayout(u8x8);
  displayMaxBmeHumidityLayout(u8x8);
}

void clearLines(int startingLine)
{
    for(startingLine; startingLine < 8; startingLine++)
    {
      u8x8.setCursor(0,startingLine);
      u8x8.print(FS(clearLine));
    }
}

#ifdef GPS_BAUD

void initGPS()
{
  gpsPort.begin(GPS_BAUD);
}

void readGPS()
{
  long readStart = millis();
  while (gps.available( gpsPort )) 
  {
    fix = gps.read();
    numSV = gps.sat_count;
    
    if(fix.valid.speed)
    {
      Speed = fix.speed_kph();
    }
    if(fix.valid.date)
    {
      year = (int)fix.dateTime.year + 2000;
      month = (int)fix.dateTime.month;
      day = (int)fix.dateTime.date;
    }
    if(fix.valid.time)
    {
      hour = (int)fix.dateTime.hours + 1;
      minutes = (int)fix.dateTime.minutes;
    }
  }

  Serial.print("GPS read time: ");
  Serial.print(millis() - readStart);
  Serial.println("ms");
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
  calculateTemp();
  calculatePressureAlt();
  calculateHumidity();
}

void calculatePressureAlt()
{
  PressureAltitude = calculateBmeAlt(); 
  
  if(PressureAltitude > MaxPressureAltitude)
    MaxPressureAltitude = PressureAltitude;
}

void calculateHumidity()
{
  Humidity = calculateBmeHumidity();
  
  if(Humidity > MaxHumidity)
    MaxHumidity = Humidity;

  if(Humidity < MinHumidity)
    MinHumidity = Humidity;
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
