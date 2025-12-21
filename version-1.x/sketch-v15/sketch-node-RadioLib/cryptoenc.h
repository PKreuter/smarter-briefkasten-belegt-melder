

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
 *   aes_encrypt(
 *      const char input*, 
 *      uint8_t *key, 
 *      unit8_t *iv);
 *
 *   return String as <AES encrypted and BASE64 encoded>
 * 
 **/
String aes_encrypt(const char *input, uint8_t *key, uint8_t *iv) {
#if defined(CRYPTO_DEBUG)
  Serial.println("---Decrypt");
#endif

  // IV, make a fresh copy!!!
  uint8_t iv_copy[16] = {0};
  memcpy(iv_copy, iv, 16);

  // AES Key size
  int key_size = strlen((char *)key);
  int plaintext_length = strlen((char *)input);

#if defined(CRYPTO_DEBUG)
  Serial.print("Key size: "); Serial.println(key_size);
  Serial.print("Plaintext size: "); Serial.println(plaintext_length);
#endif  

  int padded_input_len = 0;
  int input_len = strlen(input) + 1;
  int modulo16 = input_len % 16;
  uint8_t pkc5_value = (17 - modulo16);

  mbedtls_aes_context aes; //aes is simple variable of given type
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, key, key_size*8);//set key for encryption

  if (pkc5_value == 16) {
    int plain_length = strlen(input);
    char* plain_data = (char *)malloc(plain_length);
    memcpy(plain_data, input, strlen(input));
    unsigned char cipherText[plain_length]; 
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, plain_length, iv_copy, (unsigned char *)plain_data, cipherText);
    mbedtls_aes_free(&aes);

    //print output hex buffer on console
    int cipherTextSize = sizeof(cipherText) / sizeof(const unsigned char);

#if defined(CRYPTO_DEBUG)
    Serial.print("Size of AES encrypted output: ");
    Serial.println(cipherTextSize);  
    Serial.print("AES cihper: ");
    for (int i = 0; i < cipherTextSize; i++) {
      char str[2];
      sprintf(str, "%02x", (int)cipherText[i]);
      Serial.print(str);
    }
    Serial.println("");
    Serial.print("AES cihper as HEX: "); printHEX(cipherText);
#endif

  }
  else {
    int plaintext_length = strlen((char *)input);

#if defined(CRYPTO_DEBUG)
    Serial.print("Plaintext size: "); Serial.println(plaintext_length);
    Serial.print("Plaintext as HEX: ");
    for (int cnt = 0; cnt < plaintext_length; cnt++) {
      Serial.print(input[cnt], HEX); Serial.print(" ");
    }
    Serial.println();
#endif

    int padded_length = 0;
    int input_len = strlen(input) + 1;
    int modulo16 = input_len % 16;
    uint8_t pkc5_value = (17 - modulo16);
    padded_length = (strlen(input) / 16 + 1) * 16;

#if defined(CRYPTO_DEBUG)
    Serial.print("Modulo16: "); Serial.println(modulo16);
    Serial.print("Padded length would be: "); Serial.println(padded_length);
    Serial.print("Padding value: "); Serial.println(pkc5_value);
#endif  

    int plain_length = padded_length;
    char *plain_data = (char *)malloc(plain_length);
    memcpy(plain_data, input, plain_length);
    if (!plain_data) {
      printf("Failed to allocate memory\n");
      //return; 
    }

    unsigned char cipherText[plain_length]; 
    
    // MBEDTLS_CIPHER_PADDING_PKCS7
    for (int i = strlen(input) +1; i <= padded_length; i++) {
      plain_data[i] = pkc5_value; 
    }

#if defined(CRYPTO_DEBUG)
    Serial.println((char *)plain_data);
#endif

    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, plain_length, iv_copy, (unsigned char *)plain_data, cipherText);
    mbedtls_aes_free(&aes);
  
    //print output hex buffer on console
    int cipherTextSize = sizeof(cipherText) / sizeof(const unsigned char);

#if defined(CRYPTO_DEBUG)
    Serial.print("Size of AES encrypted output: ");
    Serial.println(cipherTextSize); 
    Serial.print("AES cihper: ");
    for (int i = 0; i < cipherTextSize; i++) {
      char str[2];
      sprintf(str, "%02x", (int)cipherText[i]);
      Serial.print(str);
    }
    Serial.println("");
    Serial.print("AES cihper as HEX: "); printHex(cipherText);
#endif

  // encode BASE64
  unsigned char output[2*BUFFER_LIMIT] = {0};
  size_t output_len;
  mbedtls_base64_encode((unsigned char *)output, (2*BUFFER_LIMIT), &output_len, (const unsigned char *)cipherText, cipherTextSize);

#if defined(CRYPTO_DEBUG)
  Serial.print("BASE64 encoded as HEX: ");
  printHex(output);
#endif

  // return aes/base64 encoded
  return ((char*)output);

  }
}
