
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


// Sensitive configs
#include "secrets.h"   
#include "config.h"

#include "esp_log.h"
#define CONFIG_LOG_DEFAULT_LEVEL ESP_LOG_INFO

#include "helpers.h"
#include "cryptodec.h"
// phaseout
#include "aes.h"

//Libraries for Communications
#include <Wire.h>
#include <SPI.h>

// https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>
int countdownMS = Watchdog.enable(60 * 1000);

// Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1351.h>

// Webserver
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

// Radio, https://github.com/jgromes/RadioLib/tree/master
#include <RadioLib.h>

#include <WiFi.h>

// to get time from NTP Server
#include <NTPClient.h>
#include <WiFiUdp.h>

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

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
TimerHandle_t wifiReconnectTimer;


// heartbeat
unsigned long previousMillis = 0;    // Stores last time temperature was published
unsigned long currentMillis = millis();

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

String TAG = "setup";

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

void setup() {
  Serial.begin(BAUD);

  // ESP Logging library
  pinMode(debugPin, INPUT_PULLDOWN);
  if( digitalRead(debugPin) == LOW) {
    esp_log_level_set("*", ESP_LOG_NONE);
  } else {
    esp_log_level_set("*", ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Log Level *********");
  }
  ESP_LOGI(TAG, "LoRa Gateway - Version %s", String(VERSION));

  ESP_LOGD(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGD(TAG, "Stack was set to %d bytes", getArduinoLoopTaskStackSize());
  ESP_LOGD(TAG, "Setup() running on core %s", String(xPortGetCoreID()));
  
  // Set the I2C pins before begin
  Wire.setPins(I2C_SDA, I2C_SCL); 
  Wire.begin(); // join i2c bus (address optional for master)

  //SPI default pins for Board ESP32-C6-N8
  SPI.begin(SCK, MISO, MOSI, SS);

  // watchdog, used when WiFi not comes up
  Watchdog.disable();
  Watchdog.reset(); // muss, sonst beleibt haengen

  // wgen Reason: 2 - AUTH_EXPIRE
  WiFi.disconnect();

  int countdownMS = Watchdog.enable(60 * 1000);
  ESP_LOGD(TAG, "Enable Watchdog with max countdown of %s milliseconds", String(countdownMS, DEC));

  initOLED1();
  //initOLED2();  // vertraget sich so nicht mit SX1262
  showDisplay_Version();
  sprintf(text, "oled done ... "); oled1WriteMsg(2, displayRow6, text); 

  initRadioSX();
  sprintf(text, "lora done ..."); oled1WriteMsg(2, displayRow6, text);
  
  // nicht getested
  //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  //wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  connectToWifi();
  sprintf(text, "wifi done ... "); oled1WriteMsg(2, displayRow6, text);
  
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // Initialize a NTPClient to get time, we use UTC
  timeClient.begin();

  setupDateTime();
  //showTime();
  sprintf(text, "time done ... "); oled1WriteMsg(2, displayRow6, text);

  // mqtt
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);
  connectToMqtt();
  sprintf(text, "mqtt done ... "); oled1WriteMsg(2, displayRow6, text);
 
  Watchdog.disable();
  ESP_LOGI("", "All seems Connected, disable Watchdog");

  // AES
  aes_init();
  sprintf(text, "aes done ... "); oled1WriteMsg(2, displayRow6, text);

  webserver();
  sprintf(text, "web done "); oled1WriteMsg(2, displayRow6, text);

  // setBrightness(0..255)
  RGB.setBrightness(20);
  // NeoPixel Bibliothek initialisieren
  RGB.begin();

  // clear
  sprintf(text, "            "); oled1WriteMsg(0,displayRow3, text);
  sprintf(text, "finished  "); oled1WriteMsg(2, displayRow6, text);
  delay(1000);
  update_display();  

}



//screen 1
const unsigned char PROGMEM screen1 [] = {
 
};




// initialize OLED SSD1306 with the I2C addr 0x3D (for the 128x64)
// bool:reset set to TRUE or FALSE depending on you display
void initOLED1() {
  ESP_LOGI(TAG, "Initializing SSD1306...");
  if (!oled1.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, true)) { 
    ESP_LOGE(TAG, "* allocation failed");
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
  Watchdog.reset(); 
 }





// Initialize LoRa Module SX1262
void initRadioSX() {
  ESP_LOGI(TAG, "Initializing SX1262...");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "OK");
  } else {
    ESP_LOGE(TAG, "failed, code %s", String(state));
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
    ESP_LOGI(TAG, "OK");
  } else {
    ESP_LOGE(TAG, "failed, code %s", String(state));
    while (true) { delay(10); }
  }

  // if needed, 'listen' mode can be disabled by calling any of the following methods:
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.scanChannel();

}


int wifi_status = WL_IDLE_STATUS;
void connectToWifi() {
  ESP_LOGI(TAG, "Connecting to WiFi %s", String(SECRET_WIFI_SSID));
  while (WiFi.status() != WL_CONNECTED) {
    ESP_LOGI(TAG, "Attemping to connect...");
    wifi_status = WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
    delay(10000);
  }
  ESP_LOGD(TAG, "%s", String(wifi_status));
  int state = WiFi.status();
  ESP_LOGD(TAG, "state %s", String(state));
  Watchdog.reset();
}


// print ip to display
void showDisplay_WifiIp() {
  oled1.setCursor(0,10);
  oled1.print("IP: ");
  oled1.setCursor(20,10);
  oled1.print(WiFi.localIP());
  oled1.display();
}


void WiFiEvent(WiFiEvent_t event) {
  ESP_LOGD("wifi", "event: %s", String(event));
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      ESP_LOGI("wifi", "Wifi connected, RSSI: %s, IP address: %s", String(WiFi.RSSI()), String(WiFi.localIP().toString()));
      showDisplay_WifiIp();
      //rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGE("wifi", "lost connection!");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
    //case SYSTEM_EVENT_STA_DISCONNECTED:
    //  Serial.println("WiFi lost connection");
    //  xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		//  xTimerStart(wifiReconnectTimer, 0);
    //  break;
  }
}


void connectToMqtt() {
  ESP_LOGI("mqtt", "Connecting to %s:%s", String(MQTT_BROKER), String(MQTT_PORT));
  mqttClient.connect();
  // loop until we are connected
  while (!mqttClient.connected() ) {
    Serial.println(mqttClient.connected());
    delay(1 * mS_TO_S_FACTOR); 
  }
  // send online message to MQTT
  uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("true").c_str());
  ESP_LOGI("mqtt", "Publishing to topic: %s, packetid: %s", String(MQTT_PUB_GW_ONLINE).c_str(), String(packetIdPub));
}

void onMqttConnect(bool sessionPresent) {
  ESP_LOGI("mqtt", "Connected, Session present: %s", String(sessionPresent));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  ESP_LOGE("mqtt", "Disconnected from MQTT!");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  ESP_LOGI("mqtt", "Received acknowledged for packetId %s", String(packetId));
  acknowledgedMsgCount = (packetId);
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
    data["DateTime UTC"] = formattedDate;
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
  String TAG = "cipher";

  json_val = {};

  // plain envelope 'data'
  JsonDocument doc;   
  if(lora_data_str.indexOf("data") > 0) {
      ESP_LOGD(TAG, "Plain data received, Plaintext length: %s", String(lora_data_str.length()));
      ESP_LOGD(TAG, "Plaintext: %s", String(lora_data_str));
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      json_val = doc["data"];
  }
  // Error message
  else if(lora_data_str.indexOf("ERROR") > 0) {
      ESP_LOGD(TAG, "ERROR message received, Message length: %s", String(lora_data_str.length()));
      ESP_LOGD(TAG, "Message: %s", String(lora_data_str));
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      // put in envelope data
      json_val = doc["data"];
    } 

  // encrypted
  else {
      ESP_LOGD(TAG, "Encrypted data received, Ciphertext length: %s", String(lora_data_str.length()));
      ESP_LOGD(TAG, "Ciphertext: %s", String(lora_data_str).c_str());
      if( digitalRead(debugPin) == HIGH) {
        Serial.println("---");
        Serial.println(lora_data_str);
      }
      sprintf(ciphertext, "%s", lora_data_str.c_str());
      byte aes_iv1[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6 }; 
      String decrypted = decrypt_impl(ciphertext, aes_iv1);

      // Check for lora messages from node 0xA1 which use aeslib.h or lora.h
      if(decrypted.indexOf("0xA1") > 0 ) {
        /**
        ESP_LOGD(TAG, "***we use radiolib rule");
        byte aes_iv[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6 }; 
        String decrypted = decrypt_impl(ciphertext, aes_iv);
        //String decrypted = decrypt(ciphertext);
        **/
        if( digitalRead(debugPin) == HIGH) {
          ESP_LOGD(TAG, "Data %s" ,String(decrypted).c_str());
          Serial.println(decrypted);
        }
        JsonDocument doc2;
        deserializeJson(doc2, decrypted);
        json_val = doc2;
      }


  // message is mbedtls encrypted
  else {
    ESP_LOGD(TAG, "Encrypted data received, Ciphertext length: %s", String(lora_data_str.length()));
    ESP_LOGD(TAG, "BASE64 encoded: %s", String(lora_data_str).c_str());

    // mbedtls, decode base64 and decrypt aes-cbc
    String plaintext = aes_decrypt((const char *)lora_data_str.c_str(), (uint8_t *)aes_key, aes_iv);

    if( digitalRead(debugPin) == HIGH) {
      ESP_LOGI(TAG, "Plaintext: %s\n", plaintext.c_str());
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

}




void processForSending() {
  String TAG = "process";

  if(!json_val.isNull()) {
    // add fields from gateway
    doc["ts"] = DateTime.now();
    doc["ts_string"] = String(DateTime.toISOString().c_str());
    doc["lora_rssi"] = String(radio.getRSSI());
    doc["lora_snr"] = String(radio.getSNR());
    doc["msg_num"] = payloadMsgCounter;
    // add data from LoRa
    doc["data"] = json_val;
    serializeJson(doc, jsonSerial);
    
    lastReceivedLoraRSSI = String(radio.getRSSI());

    // extract sender node from data
    String loraRecvFrom = json_val["node"];
    ESP_LOGD(TAG, " ***from %s", String(loraRecvFrom));
    // get length to remove header
    int x = loraRecvFrom.length();
    String str = loraRecvFrom.substring(2,4);
    str.toLowerCase();
    loraRecvFrom = str;
    lastReceivedLoraNode = loraRecvFrom;

    loraRecvFrom = "/" + loraRecvFrom;
    String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
    uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
    ESP_LOGD("mqtt", "Publishing to topic: %s, PacketId: %s, Message: %s", String(topic).c_str(), String(packetIdPub), String(jsonSerial).c_str()); 
    payloadMsgCounter++;

  }
  else {
    ESP_LOGE(TAG, "Message seems corrupt");
  }

}


// main
void loop() {
  if(receivedFlag) {            // check if the flag is set
    loraPacketRecv++;
    ESP_LOGD("SX1262", "received flag true, packet %s", String(loraPacketRecv));
  
    //RGB.fill(RGB.Color(0, 0, 255), 0, AnzahlLED);  // blau
    RGB.fill(RGB.Color(255, 0, 0), 0, AnzahlLED);  // gruen
    RGB.show();
  
    sprintf(text, "*"); oled1WriteMsg(100, displayRow6, text);
    sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv)); oled1WriteMsg(0,displayRow6, text);

    int state = radio.readData(lora_data_str);
    
    if (state == RADIOLIB_ERR_NONE) {
      receivedFlag = false;  // reset flag
      ESP_LOGD("SX1262", "RSSI:\t %s dBm", String(radio.getRSSI()));
      ESP_LOGD("SX1262", "SNR:\t %s dB", String(radio.getSNR()));
      ESP_LOGD("SX1262", "Packet length:\t %s byte", String(radio.getPacketLength()));
      processLoRaData();
      processForSending();
      update_display();
    }

    RGB.clear();
    RGB.show();
    sprintf(text, " "); oled1WriteMsg(100, displayRow6, text);  // was ist damit

  }

  mqtt_heatbeat(mqttHeatbeatIntervalMillis);
  blinkts(blinkLEDIntervalMillis);
  
}






// Every X number of seconds, it publishes a new MQTT liveness message
void mqtt_heatbeat(int intervalMillis) {
  if (currentMillis - previousMillis >= intervalMillis) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // send liveness
    mqttClient.publish(MQTT_PUB_GW_LIVENESS, 1, true, String("true").c_str());
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


void setupDateTime() {
  // setup this after wifi connected
  // you can use custom timeZone,server and timeout
  // DateTime.setTimeZone(-4);
  //   DateTime.begin(15 * 1000);
  DateTime.setServer("ch.pool.ntp.org");
  //DateTime.setTimeZone("CST-8");
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  } else {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    Serial.printf("Timestamp is %ld\n", DateTime.now());
  }
}


void showTime() {
  Serial.printf("TimeZone:      %s\n", DateTime.getTimeZone());
  Serial.printf("Up     Time:   %lu seconds\n", millis() / 1000);
  Serial.printf("Boot   Time:   %ld seconds\n", DateTime.getBootTime());
  Serial.printf("Cur    Time:   %ld seconds\n", DateTime.getBootTime() + millis() / 1000);
  Serial.printf("Now    Time:   %ld\n", DateTime.now());
  Serial.printf("OS     Time:   %ld\n", DateTime.osTime());
  Serial.printf("NTP    Time:   %ld\n", DateTime.ntpTime(2 * 1000L));
  Serial.println("===========");
  Serial.printf("Local  Time:   %s\n", DateTime.format(DateFormatter::SIMPLE).c_str());
  Serial.printf("ISO86  Time:   %s\n", DateTime.toISOString().c_str());
  Serial.printf("UTC    Time:   %s\n", DateTime.formatUTC(DateFormatter::SIMPLE).c_str());
  Serial.printf("UTC86  Time:   %s\n", DateTime.formatUTC(DateFormatter::ISO8601).c_str());
  Serial.println("===========");
  time_t t = time(NULL);
  Serial.printf("OS local:     %s", asctime(localtime(&t)));
  Serial.printf("OS UTC:       %s", asctime(gmtime(&t)));
}




