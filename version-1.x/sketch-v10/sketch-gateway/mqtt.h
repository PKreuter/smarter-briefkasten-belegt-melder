
// Config for MQTT

// MQTT Broker
PROGMEM const char* MQTT_BROKER = "192.168.1.10";
PROGMEM const int   MQTT_PORT = 1883;
PROGMEM const char* MQTT_CLIENT_ID = "lora-gateway";
PROGMEM const char* MQTT_TOPIC = "emqx/esp32";

// Message queue from LoRa Gateway
PROGMEM const char* MQTT_PUB_GW_ONLINE = "lora/gateway/online";
PROGMEM const char* MQTT_PUB_GW_LIVENESS = "lora/gateway/liveness";
PROGMEM const char* MQTT_PUB_GW_RSSI = "lora/gateway/rssi/wifi";
PROGMEM const char* MQTT_PUB_GW_EVENTS = "lora/gateway/events";

// Messages queue from LoRa Node
PROGMEM const char* MQTT_PUB_PREFIX = "lora/node";
PROGMEM const char* MQTT_PUB_DATA = "/events";


