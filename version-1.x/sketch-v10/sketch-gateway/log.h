

/**
2 ERROR
3 WARN
4 INFO
5 DEBUG
**/

// 
int MIN_LOG_LEVEL = 5;        // 4 = INFO

// debug   
const uint8_t debugPin = 39; 
bool debugMode = LOW;

// Mapping
String LOG_LEVEL_NAMES[] = {"OFF", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL"};


//
void debugOutput(String text, int logLevel) {
  if (MIN_LOG_LEVEL >= logLevel) {
    Serial.println("["+LOG_LEVEL_NAMES[logLevel]+ "] " + text);
  }
}


// is logger enabled
void isDebugEnabled() {
  pinMode(debugPin, INPUT_PULLDOWN);
  debugMode = digitalRead(debugPin);
  if( digitalRead(debugPin) == LOW) {
    //MIN_LOG_LEVEL = 0;
    MIN_LOG_LEVEL = 5;
  }
  debugOutput("Debug mode: " +String(debugMode)+ " - Level: " +String(MIN_LOG_LEVEL), 4);
}

