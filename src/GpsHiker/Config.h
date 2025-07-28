
#define BUTTON_INPUT 2
#define BUTTON_SWITCHED                            true // value if the button  switch has been pressed
#define BUTTON_TRIGGERED                          true // controls  interrupt handler
#define DEBOUNCE                              10  // time to wait in milli secs

volatile  bool interrupt_process_status = {
  !BUTTON_TRIGGERED                                     // start with no switch press pending,  ie false (!triggered)
};

#define ANALOG_READ_DELAY 500

//GPS
#define GPS_BAUD 9600
// #define GPS_BAUD 115200
#define GPS_MIN_SAT 3
#define GPS_READ_DELAY 125
#define TIMEZONE_OFFSET 1
#define SUMMER_TIME true

//I2C
#define OLED
#define OLED_REFRESH 500
#define OLED_SENSOR_CALIBRATION_DELAY 1000

//#define LM35
//#define TMP36
#define Sensor_DS18B20
#define DS18B20_Address {0x28, 0xD, 0x6A, 0x2, 0x26, 0x20, 0x1, 0x25}

#define BME280_Address 0x76