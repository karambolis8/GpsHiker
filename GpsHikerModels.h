struct ScreenUpdate 
{
  unsigned long lastScreenUpdate;
  bool doScreenUpdate;
  int previousScreen;
  int currentScreen;
  bool blink;
};

struct BatteryMonitor 
{
  float volts;
  bool blink;
};