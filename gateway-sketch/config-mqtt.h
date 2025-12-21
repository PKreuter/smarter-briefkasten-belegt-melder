
// Config for MQTT

// MQTT Broker
PROGMEM const char* MQTT_BROKER = "192.168.1.12";
PROGMEM const int   MQTT_PORT = 1883;
PROGMEM const char* MQTT_CLIENT_ID = "lora-gateway-new";
PROGMEM const char* MQTT_TOPIC = "lorawan/esp32";

// Message queue from LoRa Gateway
PROGMEM const char* MQTT_PUB_GW_ONLINE = "lora/gateway/new/online";
PROGMEM const char* MQTT_PUB_GW_RSSI = "lora/gateway/new/rssi/wifi";
PROGMEM const char* MQTT_PUB_GW_EVENTS = "lora/gateway/new/events";

// Messages queue from LoRa Node
PROGMEM const char* MQTT_PUB_PREFIX = "lora/node";
PROGMEM const char* MQTT_PUB_DATA = "/events";





