#include <U8x8lib.h>
#include "GpsHikerModels.h"

void displayCurrentGpsDataLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{

}

void displayCurrentGpsData(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, GpsReadouts* gpsReadouts)
{
  u8x8.setCursor(0, 2);
  u8x8.print(F("Lat: "));
  u8x8.setCursor(5, 2);
  u8x8.print(gpsReadouts->latitude, 10);
  
  u8x8.setCursor(0, 3);
  u8x8.print(F("Lon: "));
  u8x8.setCursor(5, 3);
  u8x8.print(gpsReadouts->longitude, 10);
  
  u8x8.setCursor(0, 4);
  u8x8.print(F("Sat:"));
  u8x8.setCursor(4, 4);
  if(gpsReadouts->numSV <= 9)
  {
    u8x8.print(FS(space));
  }
  u8x8.print(gpsReadouts->numSV);
  u8x8.setCursor(7, 4);
  u8x8.print(F("Head:"));
  u8x8.setCursor(12, 4);
  u8x8.print(gpsReadouts->heading);
  u8x8.setCursor(15, 4);
  u8x8.print(F("°"));
  
  u8x8.setCursor(0, 5);
  u8x8.print(F("GPS Alt: "));
  u8x8.setCursor(9, 5);
  if(gpsReadouts->altitude <= 999)
  {
    u8x8.print(FS(space));
  }
  if(gpsReadouts->altitude <= 99)
  {
    u8x8.print(FS(space));
  }
  if(gpsReadouts->altitude <= 9)
  {
    u8x8.print(FS(space));
  }
  u8x8.print(gpsReadouts->altitude);
  u8x8.print(F("m"));
}