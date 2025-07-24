#include <U8x8lib.h>

void displayCurrentGpsDataLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  return;
}

void displayCurrentGpsData(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, GpsReadouts* gpsReadouts)
{
// Latitude:
// Ranges from -90 to +90, with positive values indicating North and negative values indicating South of the equator.
// Longitude:
// Ranges from -180 to +180, with positive values indicating East and negative values indicating West of the Prime Meridian.
  u8x8.setCursor(0, 2);
  u8x8.print(F("Lat:"));
  if(gpsReadouts->latitude > 0)
  {
    u8x8.print(F(" "));
  }
  u8x8.print(gpsReadouts->latitude, 7);
  u8x8.print(gpsReadouts->latChar);
  
  u8x8.setCursor(0, 3);
  u8x8.print(F("Lon:"));
  if(gpsReadouts->longitude > 0)
  {
    u8x8.print(F(" "));
  }
  u8x8.print(gpsReadouts->longitude, 7);
  u8x8.print(gpsReadouts->lonChar);
  
  u8x8.setCursor(0, 4);
  u8x8.print(F("Sat:"));
  u8x8.setCursor(4, 4);
  if(gpsReadouts->numSV <= 9)
  {
    u8x8.print(FS(space));
  }
  u8x8.print(gpsReadouts->numSV);
  u8x8.setCursor(7, 4);
  u8x8.print(F("Head: "));
  u8x8.setCursor(12, 4);
  u8x8.print(gpsReadouts->heading, 0);
  
  u8x8.setCursor(0, 5);
  u8x8.print(F("GPS Alt: "));
  u8x8.setCursor(9, 5);
  u8x8.print(gpsReadouts->altitude);
  u8x8.print(F("m"));
}