
// sensitve configs in secrets.h

/**
BLOCK_SIZE
aes_key
aes_iv
**/


#include "AESLib.h"
AESLib aesLib;

#define BUFFER_LIMIT 260
//char cleartext[BUFFER_LIMIT] = {0};    // THIS IS BUFFER (FOR TEXT)
char ciphertext[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER FOR ENCRYPTED DATA


/**
String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}
**/


String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

// Generate IV (once)
void aes_init() {
  debugOutput("AES Init", 5);
  aesLib.gen_iv(aes_iv);
}

