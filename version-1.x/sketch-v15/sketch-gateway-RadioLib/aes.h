#include <ios>

// we put sensitve configs in secrets.h

/**
BLOCK_SIZE
aes_key
aes_iv
**/

// https://github.com/suculent/thinx-aes-lib
#include "AESLib.h"
AESLib aesLib;

#define AES_DEBUG


#define BUFFER_LIMIT 260
//char cleartext[BUFFER_LIMIT] = {0};    // THIS IS BUFFER (FOR TEXT)
char ciphertext[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER FOR ENCRYPTED DATA


// work between two Arduino based ESP32





// work between two Arduino based ESP32  
String decrypt_impl(char * msg, byte iv[] ) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // half may be enough
  aesLib.set_paddingmode(paddingMode::CMS);
    // decrypt bytes in "msg" and save the output in "decrypted"
    // param 1 = the source bytes to be decrypted
    // param 2 = the length of source bytes
    // param 3 = the destination of decrypted bytes that will be saved
    // param 4 = KEY
    // param 5 = the length of KEY bytes (16)
    // param 6 = IV
    // aesLib.decrypt(encryptedBytes, originalBytesLength, decryptedBytes, aesKey, 16, aesIv);
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key1, sizeof(aes_key1), iv);
  return String(decrypted);
}



/**
String decrypt(String encryptedBase64Text) {
  Serial.println("***we use new decrypt rule");
    // calculate the original length before it was coded into base64 string
    int originalBytesLength = base64::decodeLength(encryptedBase64Text.c_str());
    // declare empty byte array (a memory storage)
    byte encryptedBytes[originalBytesLength];
    byte decryptedBytes[originalBytesLength];
    // convert the base64 string into original bytes
    // which is the encryptedBytes
    base64::decode(encryptedBase64Text.c_str(), encryptedBytes);
    // initializing AES engine
    // Cipher Mode and Key Size are preset in AESLib
    // Cipher Mode = CBC
    // Key Size = 128
    // declare the KEY and IV
    //byte aesKey[] = { 23, 45, 56, 67, 67, 87, 98, 12, 32, 34, 45, 56, 67, 87, 65, 5 };
    byte aes_iv[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // set the padding mode to paddingMode.CMS
    //aesLib.set_paddingmode((paddingMode)0);
    aesLib.set_paddingmode(paddingMode::CMS);
    // decrypt bytes in "encryptedBytes" and save the output in "decryptedBytes"
    // param 1 = the source bytes to be decrypted
    // param 2 = the length of source bytes
    // param 3 = the destination of decrypted bytes that will be saved
    // param 4 = KEY
    // param 5 = the length of KEY bytes (16)
    // param 6 = IV
    aesLib.decrypt(encryptedBytes, originalBytesLength, 
                   decryptedBytes, aes_key, sizeof(aes_key), aes_iv);
    // convert the decrypted bytes into original string
    String decryptedText = String((char*)decryptedBytes);
    return decryptedText;
}
**/

/**
// the text encryption function
String encrypt(String inputText) {
    // calculate the length of bytes of the input text
    // an extra of byte must be added for a null character
    // a null character will be filled as a text terminator
    // so that the process will not overflow to other parts of memory    
    int bytesInputLength = inputText.length() + 1;
    // declare an empty byte array (a memory storage)
    byte bytesInput[bytesInputLength];
    // convert the text into bytes, a null char is filled at the end
    inputText.getBytes(bytesInput, bytesInputLength);
    // calculate the length of bytes after encryption done
    int outputLength = aesLib.get_cipher_length(bytesInputLength);
    // declare an empty byte array (a memory storage)
    byte bytesEncrypted[outputLength];
    // initializing AES engine
    // Cipher Mode and Key Size are preset in AESLib
    // Cipher Mode = CBC
    // Key Size = 128
    // declare the KEY and IV
    //byte aesKey[] = { 23, 45, 56, 67, 67, 87, 98, 12, 32, 34, 45, 56, 67, 87, 65, 5 };
    //byte aesIv[] = { 123, 43, 46, 89, 29, 187, 58, 213, 78, 50, 19, 106, 205, 1, 5, 7 };
    byte aes_iv[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // set the padding mode to paddingMode.CMS
    aesLib.set_paddingmode((paddingMode)0);
    // encrypt the bytes in "bytesInput" and store the output at "bytesEncrypted"
    // param 1 = the source bytes to be encrypted
    // param 2 = the length of source bytes
    // param 3 = the destination of encrypted bytes that will be saved
    // param 4 = KEY
    // param 5 = the length of KEY bytes (16)
    // param 6 = IV
    aesLib.encrypt(bytesInput, bytesInputLength, bytesEncrypted, aes_key, 16, aes_iv);
    // declare a empty char array
    char base64EncodedOutput[base64::encodeLength(outputLength)];
    // convert the encrypted bytes into base64 string "base64EncodedOutput"
    base64::encode(bytesEncrypted, outputLength, base64EncodedOutput);
    // convert the encoded base64 char array into string
    return String(base64EncodedOutput);
}
**/



// Generate IV (once)
void aes_init() {
  ESP_LOGD("setup", "Initializing AES...");
  // aesLib.gen_iv(aes_iv);
  // set the padding mode
  aesLib.set_paddingmode((paddingMode)0);
  //aesLib.set_paddingmode(paddingMode::CMS);
}

