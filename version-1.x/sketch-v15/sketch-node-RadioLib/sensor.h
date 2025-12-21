/**

  Sensor, Model Joy-IT® Ultraschall Abstandssensor
    Artikel Nr. SEN-US01
    Betriebsspannung 5V
    Betriebsstrom 15mA
  
  - wenn frei / unbelegt   = ON
  - wenn besetzt / belegt  = OFF

  return
  - sensorState = HIGH|LOW
  - sensorValue = 0 - xxxx

**/


// Generic
bool sensorState = LOW;
int sensorValue = 4095;

// Ultaschall-Sensor
const int Echo_EingangsPin = 2;                // Pos 4 / bn
const int Trigger_AusgangsPin = 4;             // Pos 2 / ws/bn

// PressButton and IR Sensor PIN
const int sensorPin = 4;    // 10K PULLDOWN


// Benoetigte Variablen werden definiert
int maximumRange = 100;
int minimumRange = 2;
long Abstand;
long Dauer;


void initSensorUS() {
  pinMode(Trigger_AusgangsPin, OUTPUT);
  pinMode(Echo_EingangsPin, INPUT);
}


void getSensorUSValue() {
  // Abstandsmessung wird mittels des 10us langen Triggersignals gestartet
  digitalWrite(Trigger_AusgangsPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger_AusgangsPin, LOW); 

  // Nun wird am Echo-Eingang gewartet, bis das Signal aktiviert wurde
  // und danach die Zeit gemessen, wie lang es aktiviert bleibt
  Dauer = pulseIn(Echo_EingangsPin, HIGH);

  // Nun wird der Abstand mittels der aufgenommenen Zeit berechnet
  Abstand = Dauer/58.2;
  ESP_LOGD("US Sensor", "US-Sensor, Abstand: %s", String(Abstand));
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (Abstand >= maximumRange || Abstand <= minimumRange) {
    // Falls nicht wird eine Fehlermeldung ausgegeben.
    ESP_LOGE("US Sensor", "US-Sensor, Abstand ausserhalb des Messbereich");
  }
  else {
    // Der berechnete Abstand wird in der seriellen Ausgabe ausgegeben
    ESP_LOGD("US Sensor", "US-Sensor, Der Abstand betraegt: %s cm", String(Abstand));
  }

  sensorValue = Abstand;
  if ( sensorValue > 1 and sensorValue < 8) {
    sensorState = HIGH;
  } 
  else {
    sensorState = LOW;
  }
  ESP_LOGI("US Sensor", "Analog IO - Sensor value: %s - Sensor state: %s", String(sensorValue), String(sensorState));

}



// Case Press-Button

// digital IO of a button "on=nicht belegt, off=belegt"
void getButtonState() {
  sensorValue = digitalRead(sensorPin);
  // check if the pushbutton is pressed, if it is, the buttonState is HIGH
  if (sensorValue == HIGH) {
    sensorState = HIGH;
  } else {
    sensorState = LOW;
  }
  ESP_LOGI("Press-Button", "Digital IO, Sensor value: %s - Sensor state %", String(sensorValue), String(sensorState));
}



// Case IR Sensor
/**
  Sharp Infrarot Distanzsensor 2Y0A21
  Messbereich: 10 - 80cm
  Versorgungsspannung: 4.5 - 5.5V
  Betriebstrom: ca. 30mA
  Analoger Ausgang: Nicht linear zur Entfernung
  Zeit pro Messung: 38.3ms ± 9.6ms
**/
float average (int * array, int len) {
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++) {
    sum += array [i] ;
  }
  ESP_LOGD("IR Sensor", "sum: %s, len: %s, avg as int %s", String(sum), String(len), String( ((int) sum) / len));
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}


// digital IO of a button "on=nicht belegt, off=belegt"
void getSensorIRState() {

  int AnalogWert;
  int anz = 10;
  int data[10];
  
  //delay(5000);

  for (byte i = 0; i < anz; i = i + 1) {
    AnalogWert=analogRead(sensorPin);
    data[i] = AnalogWert;  
    //Serial.println(AnalogWert);
  }

  //Serial.println(average(data, anz));
  sensorValue=(average(data, anz));
 
  if (sensorValue > 4000) {
    sensorState = LOW;
  }
  else if (sensorValue > 2500) {
    sensorState = HIGH;
  } else {
    sensorState = LOW;
  }
  ESP_LOGI("IR Sensor", "Analog IO, Sensor value: %s - Sensor state %s", String(sensorValue), String(sensorState));
}

