
// Generic config

// Version
PROGMEM const char* VERSION = "0.014c";
PROGMEM const char* NAME = "Gateway"; 

// address of this device
PROGMEM const byte localAddress = 0xAA;           

// Speed Serial Monitor
#define BAUD 115200

// setting for MQTT liveness message 
//PROGMEM const long intervalMillis = 10000;  // TEST, interval for liveness
PROGMEM const long intervalMillis = 300000;  // interval for liveness

#define LED_PIN 25

// Wire
#define I2C_SDA 21
#define I2C_SCL 22

// LoRa define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//LoRa 866E6 for Europe
#define BAND 866E6

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// time factor
#define mS_TO_S_FACTOR 1000     // factor for milli seconds to seconds
#define uS_TO_S_FACTOR 1000000  // factor for micro seconds to seconds 


