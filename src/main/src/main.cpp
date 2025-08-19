// Libraries
#include <Arduino.h>
#include <QTRSensors.h>

// Constants
#define ESC_PIN 33
#define CALIBRATION_BUTTON_PIN 25
#define SERVO_PIN 32

const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;
const int SERVO_LED_CHANNEL = 1;
const uint8_t SensorCount = 8;

// PID Control Constants (Placeholders)
const uint16_t setpoint = 3500;
const float Kp = 0.05;

// Global Variables
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// Function Declarations
void setESCMicroseconds(int microseconds);
void calibrateQTR();
void setServoAngle(int angle);
uint16_t getPosition();
void startupServoSweep();
float computePID();

// -------- Setup and Loop -------- //

void setup() {
  Serial.begin(9600);

  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);

  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, LED_CHANNEL);
  ledcSetup(SERVO_LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_LED_CHANNEL);

  calibrateQTR();

  setESCMicroseconds(1500);
  startupServoSweep();
}

void loop() { 
  float pidOutput = 90 + computePID();
  setServoAngle(constrain(pidOutput, 0, 180));
  delay(5);
}

// -------- End Setup and Loop -------- //

// Function Definitions
void setESCMicroseconds(int microseconds) {
  ledcWrite(LED_CHANNEL, int((double(microseconds) / 20000) * 65535));
}

void setServoAngle(int angle) {
  // Map 0-180 degrees to 1-2 ms pulse
  int pulse_us = map(angle, 0, 180, 1000, 2000);
  // Convert to 16-bit duty cycle (20ms period)
  int duty = (pulse_us * 65535) / 20000;
  ledcWrite(SERVO_LED_CHANNEL, duty);
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

uint16_t getPosition() {
  uint16_t position = qtr.readLineWhite(sensorValues);
  return position;
}

void startupServoSweep() {
  setServoAngle(90);
  delay(250);
  setServoAngle(0);
  delay(750);
  setServoAngle(180);
  delay(250);
  setServoAngle(90);
}

float computePID() {
  uint16_t position = getPosition();
  int error = setpoint - position;

  double proportional = error;

  return (Kp * proportional);
}