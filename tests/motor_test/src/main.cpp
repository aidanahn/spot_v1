#include <Arduino.h>

#define ESC_PIN 25

const int FREQUENCY = 50;
const int LEDC_CHANNEL = 0;
const int RESOLUTION = 16;

void setup() {
  Serial.begin(115200);
  ledcSetup(LEDC_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LEDC_CHANNEL);
}

void loop() {
  Serial.println("Test");
}