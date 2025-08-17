#include <Arduino.h>

#define ESC_PIN 33

const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;

void setESCMicroseconds(int microseconds);

void setup() {
  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LED_CHANNEL);
}

void loop() {
  setESCMicroseconds(1500);
}

void setESCMicroseconds(int microseconds) {
  ledcWrite(LED_CHANNEL, int((double(microseconds) / 20000) * 65535));
}