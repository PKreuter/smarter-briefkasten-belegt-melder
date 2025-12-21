/**

This is the LoRa Node Code 
- LILYGO / TTGO LORA - Model T3V1.6.1
- Send Data as JSON
- Use AES to encrypt payload

!!! Zum Download US Sensor entfernen !!!

// you should not use global defined variables of type String. The variable-type String eats up all RAM over time

RULES
- Default with DEEP_SLEEP_MODE
- Using RTC Memory to Store Data During Sleep
- Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 

External Components
- JOY-IT SEN-US01
- JOY_IT BMP280 (not used)
- DHT22

Arduion IDE
- Select Board : TTGO LoRa32-OLED
-
Chip is ESP32-PICO-D4 (revision v1.1)
Features: WiFi, BT, Dual Core, 240MHz, Embedded Flash, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz

PINs, 
  0   Boot
  2   LOW=Chip in download mode.
      Sensor Trigger
  4   Sensor US Echo / PressButton / IR
  5   LoRa SCK
  12  ?? darf nigh HIGH sein sonst (RTCWDT_RTC_RESET)
  13  Sleep PIN digital => PULLUP 10K
  14  LED Pin / MOS-FET to enabled Power for Sensors
  15  DHT Sensor
  18  LoRa CS / SS
  19  LoRa MISO
  21  SDA  Display
  22  SLC  Display
  23  LoRa RST
  25  LED green
  26  LoRa DIO0
  27  LoRa MOSI
  34  Wakeup digital
  35  interval VBAT analog
  36  Sleep PIN digital => PULLUP 10K
  39  Debug => Default LOW, ON = 10K to VCC 


  Wenn VBAT dauernd 7.27 V anzeigt siimmt was mit den Widerstand nicht
  PIN: 13, 36 via 10K to VCC

  
 
  LED
  - red     VBUS
  - blue    Battery
  - green   IO25 

  Interfaces
  - LoRa SPI
  - TF Card SPI
  - OLED IC2

*/ 

//#include "config-node-a1.h"
#include "config-node-a2.h"

// Sensitive configs
#include "secrets.h"   

// 
#include "config.h" 
//#include "log.h"
#include "init.h"
#include "sensor.h"

//https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>

//Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "oled.h"

//Libraries to create Json Documents
#include <ArduinoJson.h>

/** Define Sleep Mode 
 both HIGH = Mode 1 Deep
 others = Mode 2 'sleep'
 both LOW = Disabled as 'wait_for'
**/
const int sleepModePin1 = 13;  
const int sleepModePin2 = 36; //12; //39;   //2
// variable for store the sleepmode
int sleepMode = 1;   // 0=no sleep, 1=deep, 2=short
int wait_for = 15;   // every n seconds

// digital IO as Output
const int pwrPin =  14;  // Power Sensor

//const char* wakeUpPin = GPIO_NUM_34;
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
//#define GPIO_NUM_34 34

// battery voltage from ESP32 ADC read
// analog IO of the VBAT   
const uint8_t vbatPin = 35; 
float VBAT;

// Send LoRa packets
const int ledPin =  25;

// OLED line 0..6, writePixel (x, y, color)
int displayRow1 = 0;
int displayRow2 = 11;
int displayRow3 = 19;
int displayRow4 = 28;
int displayRow5 = 38;
int displayRow6 = 47;
int displayRow7 = 56;


// Buffer to write message to display
char text[25] = {0};


//global variables for temperature and Humidity
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
uint32_t delayMS;  // ???


// Using RTC Memory to Store Data During Sleep, so that it will not be deleted during the deep sleep
// Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 
RTC_DATA_ATTR unsigned int msgCounter = 0;
RTC_DATA_ATTR int bootCount = 0; 


// encrypt / decrypt lora messages
#include "aes.h"
#include <base64.h>


void bufferSize(char* text, int &length) {
  int i = strlen(text);
  int buf = round(i / BLOCK_SIZE) * BLOCK_SIZE;
  length = (buf <= i) ? buf + BLOCK_SIZE : length = buf;
}


void showVersion() {
  sprintf(text, "Version %s", String(VERSION));
  oledWriteMsg(0, text);
  sprintf(text, "LoRa Node 0x%s", String(localAddress, HEX));
  oledWriteMsg(10, text);
}


void setup() {
  //initialize Serial Monitor
  Serial.begin(BAUD);

  String TAG = "setup";
  // ESP Logging library
  pinMode(debugPin, INPUT_PULLDOWN);
  if( digitalRead(debugPin) == LOW) {
    esp_log_level_set("*", ESP_LOG_NONE);
  } else {
    esp_log_level_set("*", ESP_LOG_DEBUG);
    ESP_LOGI("*", "Log Level *********");
  }
  ESP_LOGI("*", "LoRa Node - Version %s", String(VERSION));

  // Slowing down the ESP32 to 1/4 of its speed saves more energy
	setCpuFrequencyMhz(60);

  // Battery Pin as an analog input 
  pinMode(vbatPin, INPUT);
 
  // initialize Sleep Pin, digital input
  pinMode(sleepModePin1, INPUT_PULLUP);
  pinMode(sleepModePin2, INPUT_PULLUP);

  // initialize the sensor 
  if ( enableButton == true ) {
    pinMode(sensorPin, INPUT_PULLDOWN);
  }
  else if ( enableIRSensor == true ) {
    pinMode(sensorPin, INPUT_PULLDOWN);
  }
  else {
    initSensorUS();
  }

  // initialize the power pin as an output
  pinMode(pwrPin, OUTPUT);
  digitalWrite(pwrPin, HIGH); // set power on

  // initialize green LED to indicate send packets
  pinMode(ledPin, OUTPUT);

  // initialize Sensors/OLED/LoRa
  initDHT();
  if ( enableBMP == true ) {
    initBMP();
  }
  initOLED();
  showVersion();
  initLoRa();

  // AES
  aes_init();
  aesLib.set_paddingmode(paddingMode::CMS);

  //Increments boot number and prints it every reboot
  bootCount++;
  ESP_LOGI(TAG, "Boot number: %s", String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,1); //1 = High, 0 = Low
  
  // sleep to hold messages on display
  delay(1 * mS_TO_S_FACTOR);

}



void print_sleep_info() {
  if (sleepMode == 1) {
    oledWriteMsg(45, "SLEEP Mode 1 - DEEP");
    sprintf(text, "Update every %ss", String(DEEP_SLEEP));
    oledWriteMsg(55, text);
  }
  else if (sleepMode == 2) {
    oledWriteMsg(45, "SLEEP Mode 2");
    sprintf(text, "Update every %ss", String(SLEEP));
    oledWriteMsg(55, text);
  }
  else {
    oledWriteMsg(45, "SLEEP Mode 0");
    sprintf(text, "Update every %ss", String(wait_for));
    oledWriteMsg(55, text);
  }
}


//function for fetching DHT readings
void getDHTreadings() {
  String TAG = "DHT";
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
     ESP_LOGE(TAG, "*Error reading temperature!");
  }
  else {
    Temperature = event.temperature;
    ESP_LOGI(TAG, "Temperature: %s °C", String(Temperature));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    ESP_LOGE(TAG, "*Error reading humidity!");
  }
  else {
    Humidity = event.relative_humidity;
    ESP_LOGI(TAG, "Humidity: %s %", String(Humidity));
  }
}



//function for fetching BMP280 readings
void getBMPreadings() {
  //float SLP = bmp.seaLevelForAltitude(SEALEVELPRESSURE_HPA, Pressure);
  //float LLP = bmp.readAltitude(SLP);  //SEALEVELPRESSURE_HPA
  //Pressure = LLP;
  Pressure = bmp.readPressure()/100;
  ESP_LOGI("BMP", "Pressure: %s hPa", String(Pressure));

  Temperature = bmp.readTemperature();
  ESP_LOGI("BMP", "Temperature: %s °C", String(Temperature));
}



void getSleepModeState() {

  int sleepModeValue1 = 0;
  int sleepModeValue2 = 0;
  sleepModeValue1 = digitalRead(sleepModePin1);
  sleepModeValue2 = digitalRead(sleepModePin2);

  ESP_LOGD("Sleep Mode", "Sleep PIN: %s - Value %s", String(sleepModePin1), String(sleepModeValue1));
  ESP_LOGD("Sleep Mode", "Sleep PIN: %s - Value %s", String(sleepModePin2), String(sleepModeValue2));

  if (sleepModeValue1 == HIGH and sleepModeValue2 == HIGH ) {
    sleepMode = 1;
    ESP_LOGD("Sleep Mode", "enabled, Mode %s", String(sleepMode));
  } 
  else if (sleepModeValue1 == LOW and sleepModeValue2 == LOW) {
    sleepMode = 0;
    ESP_LOGD("Sleep Mode", "disabled");
    } 
    else { 
      sleepMode = 2;
      ESP_LOGD("Sleep Mode", "enabled, Mode %s", String(sleepMode));
    }

}


/*
Method to print the reason by which ESP32 has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : ESP_LOGD("Wakeup", "caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : ESP_LOGD("Wakeup", "caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : ESP_LOGD("Wakeup", "caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : ESP_LOGD("Wakeup", "caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : ESP_LOGD("Wakeup", "caused by ULP program"); break;
    default : ESP_LOGD("Wakeup", "was not caused by deep sleep: %s", String(wakeup_reason)); break;
  }
}



//Display Readings on OLED
void displayReadings() {
  display.clearDisplay();
  oledWriteMsg(displayRow1, "RUNNING... Status: OK");
  sprintf(text, "Temperature : %sC", String(Temperature));
  oledWriteMsg(displayRow2, text);
  sprintf(text, "Humidity(%) : %sRh", String(Humidity));
  oledWriteMsg(displayRow3, text);
  sprintf(text, "Pressure : %s mmHg", String(Pressure));
  oledWriteMsg(displayRow4, text);
  sprintf(text, "Battery : %s Volts", String(VBAT));
  oledWriteMsg(displayRow5, text);

  // common rule
  if (sensorState == LOW && sensorValue == 0) {
    sprintf(text, "Post : ERROR / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text);
  }
  else if (sensorState == HIGH) {
    sprintf(text, "Post : TRUE / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text);    
  } else {
    sprintf(text, "Post : FALSE / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text); 
  }

  sprintf(text, "LoRa send :%s", String(msgCounter));
  oledWriteMsg(displayRow7, text); 
}

void getVbat() {
  // Battery Voltage
  VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
  /*
  The ADC value is a 12-bit number, so the maximum value is 4095 (counting from 0).
  To convert the ADC integer value to a real voltage you’ll need to divide it by the maximum value of 4095,
  then double it (note above that Adafruit halves the voltage), then multiply that by the reference voltage of the ESP32 which 
  is 3.3V and then vinally, multiply that again by the ADC Reference Voltage of 1100mV.
  */
  ESP_LOGD("Battery", "%s Volts", String(VBAT)); 
}


//Send data to receiver node using LoRa
void sendReadings() {
  msgCounter++;

  // localAddress as 0xAA
  char nodeAddress[10];
  sprintf(nodeAddress, "0x%02X", localAddress);

  JsonDocument doc;
  char jsonSerial[230];
  JsonObject doc1 = doc.to<JsonObject>();

  doc1["node"] = nodeAddress;
  doc1["msg_num"] = msgCounter;
  doc1["temperature"] = Temperature;
  doc1["humidity"] = Humidity;
  //doc1["pressure"] = Pressure;

  doc1["sensor_value"] = sensorValue;
  if (sensorState == LOW ) {
    doc1["sensor_state"] = "false";  // 
  }
  else {
    doc1["sensor_state"] = "true";  // Ist da
  }
  
  doc1["vbattery"] = VBAT;
  doc1["wakeup_reason"] = esp_sleep_get_wakeup_cause();

  // serialize
  serializeJson(doc, jsonSerial);

  // Encrypt
  byte enc_iv[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt_impl(jsonSerial, enc_iv);

  // send ERROR message wenn Packet > 256
  if(encrypted.length() > 240) {
    ESP_LOGE("Radio", "LoRa Packet to big");
    JsonDocument doc;
    char jsonSerial[230];
    JsonArray data = doc["data"].to<JsonArray>();
    JsonObject doc1 = data.createNestedObject();
    doc1["node"] = nodeAddress;
    doc1["msg_num"] = msgCounter;
    doc1["message"] = "ERROR LoRa Packet to big";
    serializeJson(doc, encrypted);
  }

  ESP_LOGD("Ciphertext", "Data %s ", encrypted);
  if( digitalRead(debugPin) == HIGH) {
    Serial.println(String(encrypted));
  }
  ESP_LOGD("Ciphertext", "Length %s ", String(encrypted.length()));
  

  //Send LoRa packet to receiver
  ESP_LOGI("Lora", "begin to send packet to Gateway");
  digitalWrite(ledPin, HIGH); 
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(byte(msgCounter));         // add message ID
  LoRa.write(encrypted.length());        // add payload length
  LoRa.print(encrypted);                 // add payload
  LoRa.endPacket();
  //LoRa.endPacket(true); // true = async / non-blocking mode
  digitalWrite(ledPin, LOW); 

  ESP_LOGI("Lora", "send packet done, message number: %s", String(msgCounter));

  displayReadings();

}


// prepare for sleep
void do_ready_for_sleep() {
  display.dim(true);
  display.clearDisplay();
  LoRa.sleep();
  LoRa.idle(); 
  LoRa.end();                    // set standby mode
  digitalWrite(pwrPin, LOW);    // disable Power US   
}


//function for fetching All readings at once
void getReadings() {
  getDHTreadings();
  if ( enableBMP == true ) {
    getBMPreadings();
  }
  /** Sensor enable one of them **/
  if ( enableButton == true ) {
    getButtonState();     // as digital IO based on Press-Button
  }
  else if ( enableIRSensor == true ) {
    getSensorIRState();   // as analog IO
  }
  else {
    getSensorUSValue();   // as analog IO
  }
  getVbat();
  if (sleepMode > 0) {
    digitalWrite(pwrPin, LOW);    // disable Power US 
  } 
}




void loop() {

  ESP_LOGI("main", "---");

  getReadings();
  sendReadings();

  // sleep to hold messages on display
  delay(2 * mS_TO_S_FACTOR); 
  // check sleep mode state
  getSleepModeState();

  // check for sleep mode
    /**
    Nomalbetrieb ~40 mA
    Deep Sleep Mode mit Display=on ~10mA
    Deep Sleep Mode ~5 mA
    Zuseatzlich mit LoRa.sleep() ~2mA
    Wenn Sensor mit Power dann switch-off sensor = ~2mA
    **/
  if (sleepMode == 1) {
    ESP_LOGI("main", "Go into deep sleep mode for %s seconds", String(DEEP_SLEEP));
    display.clearDisplay();
    print_sleep_info();
    delay(1 * mS_TO_S_FACTOR); 
    do_ready_for_sleep();
    esp_sleep_enable_timer_wakeup((DEEP_SLEEP -4) * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  } 
  else if (sleepMode == 2) {
    ESP_LOGI("main", "Go into deep sleep mode for %s seconds", String(SLEEP));
    display.clearDisplay();
    print_sleep_info();;
    delay(1 * mS_TO_S_FACTOR); 
    do_ready_for_sleep();
    esp_sleep_enable_timer_wakeup((SLEEP -4) * uS_TO_S_FACTOR);
    esp_deep_sleep_start(); 
  } 
  else {
    delay((5) * mS_TO_S_FACTOR);  // show display for +5 secondsdelay
    display.clearDisplay();
    print_sleep_info();;
    delay((wait_for -9) * mS_TO_S_FACTOR);  // not sleep but wait some seconds
  }

}

