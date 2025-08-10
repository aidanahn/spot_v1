#include <Arduino.h>

#define ESC_PIN 25

const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;

void setup() {
  Serial.begin(115200);
  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LED_CHANNEL);
}

void loop() {
  ledcWrite(LED_CHANNEL, 4915);
  Serial.println("Neutral");
  delay(1000);
}
