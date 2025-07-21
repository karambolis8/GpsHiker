#include <U8x8lib.h>

void displayBmeAlt(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int PressureAltitude)
{  
  u8x8.setCursor(8,5);
  
  if(PressureAltitude <= 999)
    u8x8.print(FS(space));
  
  if(PressureAltitude <= 99)
    u8x8.print(FS(space));
  
  if(PressureAltitude <= 9)
    u8x8.print(FS(space));
  
  u8x8.print(PressureAltitude);
}

void displayBmeHumidity(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int humidity)
{  
  u8x8.setCursor(9,4);
  
  if(humidity <= 9)
    u8x8.print(FS(space));
  
  u8x8.print(humidity);
}

void displayCurrentBmeAltLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,5);
  u8x8.print(F("Bar Alt:"));
  u8x8.setCursor(12,5);
  u8x8.print(F("m"));
}

void displayCurrentBmeHumidityLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,4);
  u8x8.print(F("Humidity:"));
  u8x8.setCursor(11,4);
  u8x8.print(F("%"));
}

void displayMaxBmeAltLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,5);
  u8x8.print(F("Max Alt:"));
  u8x8.setCursor(12, 5);
  u8x8.print(F("m"));
}

void displayMaxBmeHumidityLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,4);
  u8x8.print(F("Max hum:"));
  u8x8.setCursor(11,4);
  u8x8.print(F("%"));
}