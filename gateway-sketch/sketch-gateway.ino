
/**
* This is LoRa Receiver / Gateway Node
*
* Function: Receive LoRa Message and forward to MQTT Server
*
* ESP32C6 Dev Module
* - SX1262
* - OLED SSD1306
*
* Features
* - RadioLib
* - Mbedtls base64 / aes-cbc
* - AsyncWebServer
* - Sendet periodisch heartbeat to mqtt
*
* Support to receive LoRa packets from sender using 
* - 'radiolib.h' for ESP32 / Arduino
* - 'SX127x' Micro-Python
*
* MQTT Message
*       lora/node/a1/events : {
*        "ts":1735808108,
*        "ts_string":"2025-01-02T08:55:08+0000",
*        "lora_rssi":"-56.00",
*        "lora_snr":"10.50",
*        "data":{"node":"0xA1","msg_num":76,"temperature":21.4,"humidity":31,"sensor_value":0,"sensor_state":"false","vbattery":4.453509,"wakeup_reason":2}
*       }
*
**/


/**

ESP_LOGD   ESP-IF 
log_d      Arduion Wrapper

**/

// Sensitive configs
#include "secrets.h"   
#include "config.h"

#include "esp_log.h"
//#define CONFIG_LOG_DEFAULT_LEVEL ESP_LOG_INFO
#define CONFIG_LOG_DEFAULT_LEVEL ESP_LOG_DEBUG

#include "helpers.h"
#include "cryptodec.h"   // AES

//Libraries for Communications
#include <Wire.h>
#include <SPI.h>

// new watchdog
#include <esp_task_wdt.h>
//#define WDT_TIMEOUT 300 // Timeout in seconds
//if 1 core doesn't work, try with 2
//#define CONFIG_FREERTOS_NUMBER_OF_CORES 1
esp_err_t ESP32_ERROR;
int i = 0;
int last = millis();

// setting for MQTT liveness message and reste WDT
//const long mqttHeatbeatIntervalMillis = 60000;  // interval for liveness


// Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1351.h>

// Webserver
// https://github.com/ESP32Async/AsyncTCP
// https://github.com/ESP32Async/ESPAsyncWebServer
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"


// Radio, https://github.com/jgromes/RadioLib/tree/master
#include <RadioLib.h>

#include <WiFi.h>
int wifi_attempt_to_connect = 0;

// new
#include "time.h"
char timeL[80];


#include <ArduinoJson.h>
char jsonSerial[500];  // length of JSON
JsonDocument doc;      // to store input
JsonDocument json_val; // contains JSON from Node

#include "config-mqtt.h"

// https://registry.platformio.org/libraries/heman/AsyncMqttClient-esphome
#include <AsyncMqttClient.h>
// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
//TimerHandle_t wifiReconnectTimer;
// heartbeat
unsigned long previousMillis = 0;    // Stores last time was published


// Initialize variables to get and save LoRa data
unsigned long loraPacketRecv = 0;   // counter number of messages
// Store number of MQTT messages
unsigned long acknowledgedMsgCount = 0;
// store Lora messages to MQTT
unsigned long payloadMsgCounter = 1;
// store node name 
String lastReceivedLoraNode = "";
String lastReceivedLoraRSSI = "";
// store data from lora
String lora_data_str = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// debug   
const uint8_t debugPin = 23; 
bool debugMode = LOW;

// log tags
String TAG = "setup";
String APP = "app";

// OLED line 1..6, writePixel (x, y, color)
int displayRow1 = 0;
int displayRow2 = 11;
int displayRow3 = 21;
int displayRow4 = 31;
int displayRow5 = 41;
int displayRow6 = 51;
int displayRow7 = 56;

// Buffer to write message to serial port
char text[500] = {0};

WiFiClient espClient;


#include "mbedtls/base64.h"
//#define MBEDTLS_CIPHER_MODE_WITH_PADDING 
//#define MBEDTLS_CIPHER_PADDING_PKCS7  
#include "mbedtls/aes.h"
//char aes_key[] = "NIjSqCYwgXxdnXBA";
//uint8_t aes_iv[]  = {0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x30,0x31,0x32,0x33,0x34,0x35,0x36};
//#define BUFFER_LIMIT 260
char b64dectext[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER FOR BASE64 ENCRYPTED DATA
//char cipherText[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER FOR DECRYPTED DATA



// ESPDateTime: https://blog.mcxiaoke.com/ESPDateTime/index.html
#include <DateTime.h>

// Radio Module SX1262 / LoRa
SX1262 radio = new Module(SPI_RADIO_SS, SPI_RADIO_DIO1, SPI_RADIO_RST, SPI_RADIO_BUSY);
// flag to indicate that a packet was received, volatile bool receivedFlag = false;
bool receivedFlag = false;
// this function is called when a complete packet is received by the module
// IMPORTANT: this function MUST be 'void' type  and MUST NOT have any arguments!
void ICACHE_RAM_ATTR setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

// LoRa packet indicator
const int ledPin = 8;  // The ESP32-C6 pin connected to the built-in RGB LED


#include "Adafruit_NeoPixel.h"
#define RGBLED 8
#define AnzahlLED 1
// RGB -> Name der RGB-LED
Adafruit_NeoPixel RGB(AnzahlLED, RGBLED, NEO_GRB + NEO_KHZ800);


// Define OLED instance
#define OLED_RESET -1
#define SSD1306_I2C_ADDRESS 0x3c
Adafruit_SSD1306 oled1(SCREEN_1_WIDTH, SCREEN_1_HEIGHT, &Wire, OLED_RESET); // Instanziierung
#include "oled.h"



void showDisplay_Version() {
  sprintf(text, "Version %s", String(VERSION));
  oled1WriteMsg(0,displayRow1, text);
  oled1WriteMsg(0,displayRow2, "LoRa Gateway");
  delay(2 * mS_TO_S_FACTOR); 
}


#include "esp_heap_caps.h"

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name) {
  printf("%s was called but failed to allocate %d bytes with 0x%X capabilities. \n",function_name, requested_size, caps);
}

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = true,
    };


#include <rom/rtc.h>



int get_reset_reason(int icore) { 
   return (int) rtc_get_reset_reason( (RESET_REASON) icore);  
}

void print_reset_reason(int icore) {
  int reason = get_reset_reason(icore);
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3, Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}


String reset_reason_message = "";
String verbose_print_reset_reason(int reason) {
  switch ( reason) {
    case 1  : reset_reason_message = "Power on reset";break;
    case 3  : reset_reason_message = "Software reset digital core";break;
    case 4  : reset_reason_message = "Legacy watch dog reset digital core";break;
    case 5  : reset_reason_message = "Deep Sleep reset digital core";break;
    case 6  : reset_reason_message = "Reset by SLC module, reset digital core";break;
    case 7  : reset_reason_message = "Timer Group0 Watch dog reset digital core";break;
    case 8  : reset_reason_message = "Timer Group1 Watch dog reset digital core";break;
    case 9  : reset_reason_message = "RTC Watch dog Reset digital core";break;
    case 10 : reset_reason_message = "Instrusion tested to reset CPU";break;
    case 11 : reset_reason_message = "Time Group reset CPU";break;
    case 12 : reset_reason_message = "Software reset CPU";break;
    case 13 : reset_reason_message = "RTC Watch dog Reset CPU";break;
    case 14 : reset_reason_message = "for APP CPU, reseted by PRO CPU";break;
    case 15 : reset_reason_message = "Reset when the vdd voltage is not stable";break;
    case 16 : reset_reason_message = "RTC Watch dog reset digital core and rtc module";break;
    default : reset_reason_message = "NO_MEAN";
  }
  return reset_reason_message;
}


void setup() {
  Serial.begin(BAUD);

  Serial.println("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  
  Serial.println("CPU1 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(1));
  
  // ESP Logging library
  pinMode(debugPin, INPUT_PULLDOWN);
  if( digitalRead(debugPin) == LOW) {
    esp_log_level_set("*", ESP_LOG_NONE);
  } else {
    esp_log_level_set("*", ESP_LOG_DEBUG);
  }

  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("wifi", ESP_LOG_DEBUG);
  esp_log_level_set("AsyncMqttClient", ESP_LOG_WARN);
  //esp_log_level_set("asyncmqttclient", ESP_LOG_WARN);

  ESP_LOGI(TAG, "Startup..");
  ESP_LOGI(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
  ESP_LOGD(TAG, "Stack was set to %d bytes", getArduinoLoopTaskStackSize());
  ESP_LOGD(TAG, "Setup running on core %s", String(xPortGetCoreID()));
  ESP_LOGI(TAG, "LoRa Gateway - Version %s", String(VERSION));
  
  // Set the I2C pins before begin
  Wire.setPins(I2C_SDA, I2C_SCL); 
  Wire.begin(); // join i2c bus (address optional for master)

  //SPI default pins for Board ESP32-C6-N8
  SPI.begin(SCK, MISO, MOSI, SS);

  // Watchdog
  ESP_LOGD(TAG, "Configuring WDT..., Watchdog Timeout (in seconds) set to : %s", String(WDT_TIMEOUT));
  esp_task_wdt_deinit(); //wdt is enabled by default, so we need to deinit it first
  ESP32_ERROR = esp_task_wdt_init(&twdt_config); //enable panic so ESP32 restarts
  ESP_LOGI(TAG, "WDT Last Reset : %s", String(esp_err_to_name(ESP32_ERROR)));
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  delay(10);

  initOLED1();
  //initOLED2();  // vertraget sich so nicht mit SX1262

  initRadioSX();
  
  // nicht getested
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  //wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // event based
  //WiFi.onEvent(WiFiEvent);
  //connectToWifi();
/**
  // delete old config
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  **/

  // classic WiFi
  initWiFi();
  sprintf(text, "wifi done ... "); oled1WriteMsg(2, displayRow6, text);

  
  // Initialize a NTPClient to get time, we use UTC
  // https://ftp.fau.de/aminet/util/time/tzinfo.txt
  initTime("CET-1CEST,M3.5.0,M10.5.0/3"); // Europe/Zurich TZ CET-1CEST,M3.5.0,M10.5.0/3
  printlnLocalTime();
  sprintf(text, "time done ... "); oled1WriteMsg(2, displayRow6, text);

  connectToMqtt();

  webserver();
  sprintf(text, "web done "); oled1WriteMsg(2, displayRow6, text);
 
  // setBrightness(0..255)
  RGB.setBrightness(20);
  // NeoPixel Bibliothek initialisieren
  RGB.begin();

  // clear display
  sprintf(text, "            "); oled1WriteMsg(0,displayRow3, text);
  sprintf(text, "finished  "); oled1WriteMsg(2, displayRow6, text);
  delay(1000);
  update_display();  
  Serial.println("Setup finished");
}


// initialize OLED SSD1306 with the I2C addr 0x3D (for the 128x64)
// bool:reset set to TRUE or FALSE depending on you display
void initOLED1() {
  ESP_LOGI(TAG, "Initializing SSD1306...");
  if (!oled1.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, true)) { 
    ESP_LOGE(TAG, " * allocation failed");
    for (;;)
      ; // Don't proceed, loop forever
  }
  // Clear the buffer.
  oled1.display();
  delay(20); //omit delay to hide adafruit slashscreen, sorry!
  oled1.clearDisplay();
  oled1.display();
  testdrawline();
  sprintf(text, "OLED  OK");
  oled1WriteMsg(0,displayRow3, text);
  showDisplay_Version();
  sprintf(text, "oled done ... "); oled1WriteMsg(2, displayRow6, text); 
}


// Initialize LoRa Module SX1262
void initRadioSX() {
  ESP_LOGI(TAG, "Initializing SX1262...");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, " OK");
  } else {
    ESP_LOGE(TAG, " failed, code %s", String(state));
    while (true) { delay(10); }
  }
  // set carrier frequency to MHz
  if (radio.setFrequency(RADIO_BAND) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    ESP_LOGE(TAG, "Selected frequency is invalid for this module!");
    while (true) { delay(10); }
  }
  // set spreading factor
  if (radio.setSpreadingFactor(radioSpreadingFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    ESP_LOGE(TAG, "Selected spreading factor is invalid for this module!");
    while (true) { delay(10); }
  }
  // set LoRa sync word
  if (radio.setSyncWord(0x14) != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "Unable to set sync word!");
    while (true) { delay(10); }
  }
  // set the function that will be called, when new packet is received
  radio.setPacketReceivedAction(setFlag);
  // start listening for LoRa packets
  ESP_LOGI(TAG, "Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, " OK");
  } else {
    ESP_LOGE(TAG, " failed, code %s", String(state));
    while (true) { delay(10); }
  }
  sprintf(text, "lora done ..."); oled1WriteMsg(2, displayRow6, text);
}


// OLD
unsigned long wifi_previousMillis = 0;
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}
// END OLD


void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  log_i("WiFi connected to AP successfully!");
  showDisplay_WifiIp();
}


void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  log_i("Wifi connected, RSSI: %s, IP address: %s", String(WiFi.RSSI()), String(WiFi.localIP().toString()));
}


void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  log_e("WiFi disconnected from access point");
  //log_e("WiFi lost connection. Reason: %s");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
}


/**
void connectToWifi() {
  // delete old config
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);

  esp_task_wdt_reset();
  delay(1);  //VERY VERY IMPORTANT for Watchdog Reset to apply. At least 1 ms
  log_i("WiFi seems connected, Resetting WDT");
  sprintf(text, "wifi done ... "); oled1WriteMsg(2, displayRow6, text);
}
**/

// print ip to display
void showDisplay_WifiIp() {
  oled1.setCursor(0,10);
  oled1.print("IP: ");
  oled1.setCursor(20,10);
  oled1.print(WiFi.localIP());
  oled1.display();
}


uint32_t notConnectedCounter = 0;
void WiFiEvent(WiFiEvent_t event) {
  log_d("event: %s", String(event));
  Serial.println(event);
  /**
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      log_i("Wifi connected, RSSI: %s, IP address: %s", String(WiFi.RSSI()), String(WiFi.localIP().toString()));
      showDisplay_WifiIp();
      //rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:   // 14
      log_e("lost connection!");
      notConnectedCounter++;
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 10);
      if(notConnectedCounter > 100) {
        log_e("Resetting due to WiFi not connecteding...");
        delay(1000);
        ESP.restart();
      }
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      log_e("lost connection!");
      notConnectedCounter++;
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 10);
      break;
    //case SYSTEM_EVENT_STA_DISCONNECTED:
    //  Serial.println("WiFi lost connection");
    //  xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		//  xTimerStart(wifiReconnectTimer, 0);
    //  break;
  }
  **/
}


//

void connectToMqtt() {
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);
  
  ESP_LOGI(TAG, "Connecting to %s:%s", String(MQTT_BROKER), String(MQTT_PORT));
  mqttClient.connect();
  // loop until we are connected
  while (!mqttClient.connected() ) {
    Serial.println(mqttClient.connected());
    delay(1 * mS_TO_S_FACTOR); 
  }
  // send online message to MQTT
  uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("new").c_str());

  //String reset_reason(rtc_get_reset_reason(0));
  String reset_reason_message(verbose_print_reset_reason(rtc_get_reset_reason(1)));
  String message = "new (" +reset_reason_message+ ")";
  packetIdPub = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String(message).c_str());

 //     String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
 //   uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);

  
  log_i("Publishing to topic: %s, packetid: %s", String(MQTT_PUB_GW_ONLINE).c_str(), String(packetIdPub));

  sprintf(text, "mqtt done ... "); oled1WriteMsg(2, displayRow6, text);
  esp_task_wdt_reset();
  delay(1);  //VERY VERY IMPORTANT for Watchdog Reset to apply. At least 1 ms
  ESP_LOGI(TAG, " MQTT connected");
}

void onMqttConnect(bool sessionPresent) {
  log_i("Connected, Session present: %s", String(sessionPresent));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  log_e("Disconnected from MQTT!");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttPublish(uint16_t packetId) {
  log_i("Received acknowledged for packetId %s", String(packetId));
  acknowledgedMsgCount = (packetId);
  //esp_task_wdt_reset();
  //delay(1);  //VERY VERY IMPORTANT for Watchdog Reset to apply. At least 1 ms
}
// END mqtt



void webserver() {  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  // Health Endpoint
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument data;
    long ts = DateTime.now();
    String formattedDate = DateTime.toISOString().c_str();
    data["ts"] = ts;
    data["DateTime"] = formattedDate;
    data["Uptime seconds"] = esp_timer_get_time() / 1000000;
    data["name"] = NAME;
    data["firmware_rev"] = VERSION;
    data["wifi_rssi"] = String(WiFi.RSSI());
    JsonDocument doc_lora;
    doc_lora["Packets received"] = loraPacketRecv;
    doc_lora["Node last seen"] = lastReceivedLoraNode;
    doc_lora["RSSI last seen"] = lastReceivedLoraRSSI;
    data["lora"] = doc_lora;
    // nested
    JsonDocument doc_mqtt; 
    doc_mqtt["Server"] = MQTT_BROKER;
    doc_mqtt["Status"] = "online";
    doc_mqtt["Payload packets send"] = payloadMsgCounter;    
    doc_mqtt["Total packets acknowledged"] = acknowledgedMsgCount;
    data["mqtt"] = doc_mqtt;
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });
  server.onNotFound(notFound);
  server.begin();
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}
// END webserver


// update dispay
void update_display() {
  oled1WriteMsg(0,displayRow1, "RUNNING...    ");
  sprintf(text, "WiFi rssi: %s        ", String(WiFi.RSSI()));
  oled1WriteMsg(0,displayRow3, text);
  sprintf(text, "MQTT tx: %s        ", String(acknowledgedMsgCount));
  oled1WriteMsg(0,displayRow4, text);
  sprintf(text, "Lora last seen: %s        ", String(lastReceivedLoraNode));
  oled1WriteMsg(0,displayRow5, text);
  sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
  oled1WriteMsg(0,displayRow6, text);    
}


void processLoRaData() {
  json_val = {};

  // plain envelope 'data'
  JsonDocument doc;   
  if(lora_data_str.indexOf("data") > 0) {
      log_d("Plain data received, Plaintext length: %s", String(lora_data_str.length()));
      log_d("Plaintext: %s", String(lora_data_str));
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      json_val = doc["data"];
  }
  // Error message
  else if(lora_data_str.indexOf("ERROR") > 0) {
      log_d("ERROR message received, Message length: %s", String(lora_data_str.length()));
      log_d("Message: %s", String(lora_data_str));
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      // put in envelope data
      json_val = doc["data"];
    } 

  // message is mbedtls encrypted
  else {
    log_d("Encrypted data received, Ciphertext length: %s", String(lora_data_str.length()));
//#if defined(CRYPTO_DEBUG)
//    log_d("BASE64 encoded: %s", String(lora_data_str).c_str());
//#endif
    // mbedtls, decode base64 and decrypt aes-cbc
    String plaintext = aes_decrypt((const char *)lora_data_str.c_str(), (uint8_t *)aes_key, aes_iv);

    if( digitalRead(debugPin) == HIGH) {
      log_i("Plaintext: %s", plaintext.c_str());
    }

    JsonDocument doc2;
    DeserializationError error = deserializeJson(doc2, plaintext);
    if (error) {
      Serial.print("*deserializeJson() returned ");
      Serial.println(error.c_str());
      return;
    }
    json_val = doc2;

#if defined(DEBUG)
    pp_json(json_val);
#endif    
  }

}




void processForSending() {
  if(!json_val.isNull()) {
    // add gateway data
    doc["ts"] = DateTime.now();
    doc["ts_string"] = String(DateTime.toISOString().c_str());
    doc["lora_rssi"] = String(radio.getRSSI());
    doc["lora_snr"] = String(radio.getSNR());
    doc["msg_num"] = payloadMsgCounter;
    // add data from LoRa
    doc["data"] = json_val;
    serializeJson(doc, jsonSerial);

    // extract sender node from data
    String loraRecvFrom = json_val["node"];
    log_d(" ***from %s", String(loraRecvFrom));
    // get length to remove header
    int x = loraRecvFrom.length();
    String str = loraRecvFrom.substring(2,4);
    str.toLowerCase();
    loraRecvFrom = str;
    //lastReceivedLoraNode = loraRecvFrom;

    loraRecvFrom = "/" + loraRecvFrom;
    String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
    uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
    log_d("Publishing to topic: %s, PacketId: %s, Message: %s", String(topic).c_str(), String(packetIdPub), String(jsonSerial).c_str()); 
    payloadMsgCounter++;

    // values used in display
    lastReceivedLoraRSSI = String(radio.getRSSI());
    lastReceivedLoraNode = loraRecvFrom;

  }
  else {
    log_e("Message seems corrupt");
  }

}


// main
void loop() {

  // classsic, check Wifi
  unsigned long wifi_currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (wifi_currentMillis - wifi_previousMillis >=wifi_interval)) {
    Serial.print(millis());
    ESP_LOGW(APP, "Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    wifi_previousMillis = wifi_currentMillis;
  }

  if(receivedFlag) {   // check if the flag is set
    loraPacketRecv++;
    ESP_LOGD(APP, "received flag true, packet %s", String(loraPacketRecv));
  
    //RGB.fill(RGB.Color(0, 0, 255), 0, AnzahlLED);  // blau
    RGB.fill(RGB.Color(255, 0, 0), 0, AnzahlLED);  // gruen
    RGB.show();
  
    sprintf(text, "*"); oled1WriteMsg(100, displayRow6, text);
    sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv)); oled1WriteMsg(0,displayRow6, text);

    int state = radio.readData(lora_data_str);
    
    if (state == RADIOLIB_ERR_NONE) {
      receivedFlag = false;  // reset flag
      ESP_LOGD(APP, "RSSI:\t %s dBm", String(radio.getRSSI()));
      ESP_LOGD(APP, "SNR:\t %s dB", String(radio.getSNR()));
      ESP_LOGD(APP, "Packet length:\t %s byte", String(radio.getPacketLength()));
      processLoRaData();
      processForSending();
      update_display();
    }

    RGB.clear();
    RGB.show();
    sprintf(text, " "); oled1WriteMsg(100, displayRow6, text);  // was ist damit

    printLocalTime();
    Serial.println();
    esp_task_wdt_reset();
    delay(1);  //VERY VERY IMPORTANT for Watchdog Reset to apply. At least 1 ms

  }

  mqtt_heartbeat(mqttHeartbeatIntervalMillis);
  blinkts(blinkLEDIntervalMillis);

  delay(1);

}


// mqtt heartbeat, every X number of milliseconds
void mqtt_heartbeat(int intervalMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalMillis) {
    // save the last time a new heartbeat was published
    previousMillis = currentMillis;
    // send heartbeat
    mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("true").c_str());
  }
}


// show on display start
void testdrawline() {
  int16_t i;
  oled1.clearDisplay(); // Clear display buffer
  for(i=0; i<oled1.width(); i+=4) {
    oled1.drawLine(0, 0, i, oled1.height()-1, WHITE);
    oled1.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<oled1.height(); i+=4) {
    oled1.drawLine(0, 0, oled1.width()-1, i, WHITE);
    oled1.display();
    delay(1);
  }
  oled1.clearDisplay();
}


// blinking star on display
int stateLED = LOW;
unsigned long previousLEDmillis = 0;                      // timestamp stores time LED changed                            
void blinkts(int intervalMillis) {
  unsigned long timeNow = millis();                       // start a millis count
  if (timeNow - previousLEDmillis > intervalMillis) {     // counts up time until set interval
    previousLEDmillis = timeNow;                          // save it as new time (interval reset)
    if (stateLED == LOW)  {                               // check if LED is LOW
      stateLED = HIGH; 
      sprintf(text, "*   ");                              // then set it HIGH for the next loop
      oled1WriteMsg(100,displayRow1, text);               // Print "ON" at Serial console
    } else  {      
      stateLED = LOW;  
      sprintf(text, "    ") ;                             // in case LED is HIGH make LED LOW
      oled1WriteMsg(100,displayRow1, text);               // Print "OFF" at Serial console
    }
  }
}


// time
void setTimezone(const char * timezone) {
  log_i("Setting Timezone to %s", String(timezone).c_str());
  setenv("TZ", timezone, 1);
  tzset();
}


void initTime(const char * timezone) {
  struct tm timeinfo;
  log_i("Getting time from NTP server");
  configTime(0, 0, TIME_SERVER);    // First connect to NTP server, use 0 TZ offset
  if (!getLocalTime(&timeinfo)) {
    log_e("  Failed to obtain time");
    setTimezone(timezone); // then set your timezone
    return;
  }
  log_i("OK, Got the time from NTP");
  setTimezone(timezone); // then set your timezone
}

void printlnLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    log_e("  Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
}


void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    log_e("  Failed to obtain time");
    return;
  }
  Serial.print(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
}
  

void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst){
  struct tm tm;
  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month-1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0
  time_t t = mktime(&tm);
  Serial.printf("Setting time: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

