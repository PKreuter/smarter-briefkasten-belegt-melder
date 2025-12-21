/**
*
*
*
*
*
*
**/


#include <ArduinoJson.h>

// pretty print JSON doc
void pp_json(JsonDocument doc) {
    char buffer[500];
    serializeJsonPretty(doc, buffer);	
    Serial.println(buffer);
}


/** as HEX String
*   uint8_t       exakt 8 Bits ohne Vorzeichen.
*   uint_fast8_t  mindestens 8 Bits ohne Vorzeichen, laufzeitoptimiert.
*   char          Zeichen mit oder ohne Vorzeichen, 8 oder mehr Bits.
*                 Bei plattformunabhängigem Code ist also uint_fast8_t sinnvoll, da 
*                 uint8_t bei 16/32-Bit Prozessoren ineffizient sein kann
**/
void printHex(unsigned char *array) {
  for (int i = 0; i < strlen((char*)array); ++i) {
    printf("%02x", (int)array[i]);
  }
  printf("\n");
}


