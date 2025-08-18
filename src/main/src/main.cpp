#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

#define ESC_PIN 33
#define CALIBRATION_BUTTON_PIN 25

const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setESCMicroseconds(int microseconds);
void calibrateQTR();

void setup() {
  Serial.begin(9600);

  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);

  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LED_CHANNEL);

  calibrateQTR();
}

void loop() {
  setESCMicroseconds(1500);

  uint16_t position = qtr.readLineWhite(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}

void setESCMicroseconds(int microseconds) {
  ledcWrite(LED_CHANNEL, int((double(microseconds) / 20000) * 65535));
}

void calibrateQTR() {
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 13, 18, 19, 21, 22, 23}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}