#include <U8x8lib.h>

void displayCurrentBmeAlt(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int PressureAltitude)
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

void displayCurrentBmeAltLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,5);
  u8x8.print(F("Bar Alt:"));
  u8x8.setCursor(12,5);
  u8x8.print(F("m"));
}

void displayMaxBmeAlt(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int MaxPressureAltitude)
{
  u8x8.setCursor(8,5);  
  
  if(MaxPressureAltitude <= 999)
    u8x8.print(FS(space));
  
  if(MaxPressureAltitude <= 99)
    u8x8.print(FS(space));
  
  if(MaxPressureAltitude <= 9)
    u8x8.print(FS(space));
  
  u8x8.print(MaxPressureAltitude);
}

void displayMaxBmeAltLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0,5);
  u8x8.print(F("Max Alt:"));
  u8x8.setCursor(12, 5);
  u8x8.print(F("m"));
}