
#define BUTTON_INPUT 2

#define ANALOG_READ_DELAY 500

//RX-TX
#define GPS_BAUD 9600
// #define GPS_BAUD 115200
#define GPS_MIN_SAT 0

//I2C
#define OLED
#define OLED_REFRESH 500
#define OLED_SENSOR_CALIBRATION_DELAY 500

//#define LM35
//#define TMP36
#define Sensor_DS18B20
#define DS18B20_Address {0x28, 0xD, 0x6A, 0x2, 0x26, 0x20, 0x1, 0x25}

#define BME280_Address 0x76