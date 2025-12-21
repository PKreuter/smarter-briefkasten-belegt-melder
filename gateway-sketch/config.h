#include "pgmspace.h"


/**

Chip is ESP32-C6 (QFN40) (revision v0.0)


IDE
- ESP32C6 Dev Module
- Flash Size 8MB
- Partition Scheme 8MB
- Tools > USB CDC On Boot > Disabled

     4 : GPIO
     5 : GPIO
     6 : I2C_MASTER_SDA[0]
     7 : I2C_MASTER_SCL[0]
     8 : GPIO
    11 : GPIO
    16 : UART_TX[0]
    17 : UART_RX[0]
    19 : SPI_MASTER_MOSI[0]
    20 : SPI_MASTER_MISO[0]
    21 : SPI_MASTER_SCK[0]

**/

// Generic config



// Version
PROGMEM const char* VERSION = "x.001";
PROGMEM const char* NAME = "Gateway"; 

// address of this device
PROGMEM const byte localAddress = 0xBB;           

// Speed Serial Monitor
#define BAUD 115200

// Time
#define TIME_SERVER "ch.pool.ntp.org"
#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"

// This sets Arduino Stack Size - comment this line to use default 8K stack size
SET_LOOP_TASK_STACK_SIZE(16 * 1024);  // 16KB

// LED or Display* Intervall
PROGMEM const long blinkLEDIntervalMillis = 2000; 



//#define LED_PIN 8

// Pins used by Wire
#define I2C_SDA 6
#define I2C_SCL 7


// SPI, Default for ESP32-C6 MOSI: 19, MISO: 20, SCK: 21, SS: 18
#define MOSI 19  // SDI
#define MISO 20  // SDO, DIO                        
#define SCK  21
#define SS   10

// Pins used by OLED 128x128
#define SPI_OLED_2_SS  18  // CS
#define SPI_OLED_2_RST 1   // RST

// OLED 128x128 und SX1262 vertragen sich nicht

// Pins used by Radio SX1262 
#define SPI_RADIO_SS   4
#define SPI_RADIO_DIO1 11
#define SPI_RADIO_RST  5
#define SPI_RADIO_BUSY 3


//LoRa band, 866E6 for Europe
//#define RADIO_BAND 866.6
#define RADIO_BAND 866

/*
Der Spreading Factor (SF) beschreibt dabei wieviele Chirps, also Daten Carrier pro Sekunde übertragen werden. 
Dadurch ist die Bitrate, pro Symbol abgestrahlte Energie und die Reichweite definiert. 
Dabei wurde bei LoRa die „Adaptive Data Rate“ eingeführt: 
je nach Netzwerk Konfiguration wird über den Sender der Spreading Faktor zwischen SF7 und SF12 eingestellt.
SF7 = range 2km, time on air 61ms, 5470bps
**/
#define radioSpreadingFactor 7

// OLED 128x64
#define SCREEN_1_WIDTH 128
#define SCREEN_1_HEIGHT 64

// OLED 128x128
#define SCREEN_2_WIDTH  128
#define SCREEN_2_HEIGHT 128 


// time factor
#define mS_TO_S_FACTOR 1000     // factor for milli seconds to seconds
#define uS_TO_S_FACTOR 1000000  // factor for micro seconds to seconds 


// watchdog
#define WDT_TIMEOUT 300 // Timeout in seconds
//#define WDT_TIMEOUT 900 // Timeout in seconds
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1

// WiFi check connections
unsigned long wifi_interval = 30000;

// MQTT heartbeat online message
const long mqttHeartbeatIntervalMillis = 60000;  // interval for liveness



