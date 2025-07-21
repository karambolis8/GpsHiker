#include <U8x8lib.h>
#include "GpsHikerModels.h"

void printBattery(U8X8_SSD1306_128X64_NONAME_HW_I2C& u8x8, BatteryMonitor* battery)
{
  if(battery->blink)
  {
    u8x8.setInverseFont(1);
  }

  u8x8.setCursor(7, 1);
  u8x8.print(F("Batt: "));
  u8x8.print(battery->volts, 1);
  u8x8.print(F("v"));

  u8x8.setInverseFont(0);
}

void calculateBattery(BatteryMonitor* battery)
{
  battery->volts = 4.2;

  if(battery->volts <= 2.7 && !battery->blink)
  {
    battery->blink = true;
  }
  else
  {
    battery->blink = false;
  }
}