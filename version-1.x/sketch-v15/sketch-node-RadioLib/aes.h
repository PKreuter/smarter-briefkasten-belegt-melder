

#include "AESLib.h"
AESLib aesLib;

#define BUFFER_LIMIT 300
char cleartext[BUFFER_LIMIT] = {0};    // THIS IS BUFFER (FOR TEXT)
char ciphertext[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER (FOR BASE64-ENCODED ENCRYPTED DATA)

unsigned long loopcount = 0;


/* non-blocking wait function */
void wait(unsigned long milliseconds) {
  unsigned long timeout = millis() + milliseconds;
  while (millis() < timeout) {
    yield();
  }
}


String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}


String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

// Generate IV (once)
void aes_init() {
  ESP_LOGD("AES", "Init...");
  //aesLib.gen_iv(aes_iv);
}



