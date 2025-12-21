
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


// AES settings, you must use your own KEY / IV for full security!!!
// AES Encryption Key
char aes_key[] = "1234567890123456";
// General initialization vector
uint8_t aes_iv[]  = {0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x30,0x31,0x32,0x33,0x34,0x35,0x36};

