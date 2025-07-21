#include <U8x8lib.h>

void displayCurrentTemp(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int T1)
{
  u8x8.setCursor(5, 7);  
  if((T1 <= 99 && T1 > 0) || (T1 < 0 && abs(T1) <= 9))
    u8x8.print(FS(space));
  if(T1 > 0 && T1 <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1);
}

void displayCurrentTempLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0, 7);
  u8x8.print(F("Tmp:"));
  u8x8.setCursor(8, 7);  
  u8x8.print(F("C"));
}

void displayMaxTemp(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, int T1Max)
{
  u8x8.setCursor(9, 7);
  if((T1Max <= 99 && T1Max > 0) || (T1Max < 0 && abs(T1Max) <= 9))
    u8x8.print(FS(space));
  if(T1Max > 0 && T1Max <= 9)
    u8x8.print(FS(space));
  u8x8.print(T1Max);
}

void displayMaxTempLayout(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8)
{
  u8x8.setCursor(0, 7);
  u8x8.print(F("Max Tmp:"));
  u8x8.setCursor(12, 7); 
  u8x8.print(F("C"));
}