#include <Arduino.h>

#define ESC_PIN 25

const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;

void setESCMicroseconds(int microseconds) {
  ledcWrite(LED_CHANNEL, int((double(microseconds) / 20000) * 65535));
}

void setup() {
  Serial.begin(115200);
  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LED_CHANNEL);
  setESCMicroseconds(1500);
  Serial.println("Turn on ESC");
  delay(1000);
  Serial.println("Starting Test");
}

void loop() {
  setESCMicroseconds(1565);
  delay(1000);
  while (true) {
    setESCMicroseconds(1500);
    Serial.println("Test Finished");
  }
  
}


