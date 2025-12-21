

// enable for debugging
//#define CRYPTO_DEBUG


#define MBEDTLS_CIPHER_MODE_WITH_PADDING 
#define MBEDTLS_CIPHER_PADDING_PKCS7  

#include "mbedtls/base64.h"
#include "mbedtls/aes.h"


#define BUFFER_LIMIT 260
char output[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER FOR BASE64 ENCRYPTED DATA








/**
 * 
 *   aes_decrypt(
 *      const char *input as <BASE64 encoded and AES encrypted>, 
 *      uint8_t *key, 
 *      uint8_t *iv);
 *
 *   return String
 * 
 **/
String aes_decrypt(const char * input, uint8_t *key, uint8_t *iv) {

  // as input we expect base64
#if defined(CRYPTO_DEBUG)
  Serial.print("BASE64 encoded: ");
  Serial.println(input);
#endif

  // step 1, decode base64
  size_t input_len = strlen(input);
  unsigned char cipherText[2*BUFFER_LIMIT] = {0};
  size_t cipherText_len;
  mbedtls_base64_decode((unsigned char *)cipherText, (2*BUFFER_LIMIT), &cipherText_len, (const unsigned char *)input, input_len);

  int cipherTextSize = sizeof(cipherText) / sizeof(const unsigned char);

#if defined(CRYPTO_DEBUG)
  Serial.print("ciperText length: ");
  Serial.println(cipherText_len);
  Serial.print("cipherText_Size: ");
  Serial.println(cipherTextSize);
#endif

  // nehmen wir korrekte laenge, geht aktuell nicht direkt
  int ct_len = cipherText_len;

#if defined(CRYPTO_DEBUG)
  Serial.print("AES cihper: ");
  for (int i = 0; i < ct_len; i++) {
    char str[2];
    sprintf(str, "%02x", cipherText[i]);
    Serial.print(str);
  }
  Serial.println("");
  Serial.print("AES cihper as HEX: ");
  for (int i = 0; i < ct_len; i++) {
    Serial.print(cipherText[i], HEX); Serial.print("");
  }
  Serial.println();
  Serial.println("---");
  // TEST
  //Serial.print("AES cihper as HEX: "); printHex(cipherText);
#endif

  // decrypt

  // IV, make a fresh copy!!!
  uint8_t iv_copy[16] = {0};
  memcpy(iv_copy, aes_iv, 16);

  int aes_key_size = strlen((char *)aes_key);

  uint8_t output[ct_len] = {0};

  mbedtls_aes_context aes1;
  mbedtls_aes_init(&aes1);
  mbedtls_aes_setkey_dec(&aes1, (uint8_t *)aes_key, aes_key_size*8);
  mbedtls_aes_crypt_cbc(&aes1, MBEDTLS_AES_DECRYPT, ct_len, iv_copy, (unsigned char *)cipherText, output);
  mbedtls_aes_free(&aes1);

  int plainTextSize = sizeof(output) / sizeof(const unsigned char);

#if defined(CRYPTO_DEBUG)
  Serial.print("Plaintext size: "); Serial.println(plainTextSize);
  Serial.print("Decrypted text: ");
  for (int i = 0; i < plainTextSize; i++) {
    char str[2];
    sprintf(str, "%02x", (int)output[i]);
    Serial.print(str);
  }
  Serial.println("");
  Serial.print("Decrypted text as HEX: ");
  printHex(output);
  Serial.printf("Decrypted text: %s\n", (char*)output);
# endif

  return (char*)(output);
}

