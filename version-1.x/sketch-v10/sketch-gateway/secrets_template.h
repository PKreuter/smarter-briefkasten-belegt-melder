
/**
Sensitive config needs to stored in file 'secrets.h'
You must use your own settings!!!
**/

// Network credentials
const char* SECRET_WIFI_SSID = "ssid";      
const char* SECRET_WIFI_PASSWORD = "password"; 


// MQTT credentials
const char* SECRET_MQTT_USERNAME = "username";
const char* SECRET_MQTT_PASSWORD = "password";


/**
AES settings, you must use your own KEY / IV for full security!!!
**/
#define BLOCK_SIZE 16
// AES Encryption Key
PROGMEM byte aes_key[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// General initialization vector
PROGMEM byte aes_iv[BLOCK_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


