#include <Arduino.h>

#define FS(x) (__FlashStringHelper*)(x)

const char clearLine[] PROGMEM = { "                " };
const char space[] PROGMEM = { " " };