#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AS5600.h>
#include <PID_v1.h>
#include <QTRSensors.h>

const char* ssid = "PacerESP32";
const char* password = "04082008";

WebServer server(80);

// Objects
AS5600 as5600;
QTRSensors qtr;

// Pin Definitions
#define ESC_PIN 33
#define SERVO_PIN 32
#define BUTTON_PIN 14

// Motor/Servo/Sensor Constants
const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;
const int SERVO_LED_CHANNEL = 1;
const int NEUTRAL_THROTTLE = 1500;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Distance Calculation Constants
const double GEAR_RATIO = (40.0 / 20.0) * (38.0 / 13.0);
const double WHEEL_DIAMETER = 121.9 / 1000.0;
const double WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Website Inputs
double distance = 0.0;
double goalTime = 0;
int microsecondsTemporary = NEUTRAL_THROTTLE;

// Steering PID Values
double Kp = 0.0;
double Kd = 0.0;
double Ki = 0.0;

// Motor PID Values
double motorKp = 0.0;
double motorKd = 0.0;
double motorKi = 0.0;

// PID Variables
double Setpoint, Input, Output;
PID steeringPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double MotorSetpoint, MotorInput, MotorOutput;
PID motorPID(&MotorInput, &MotorOutput, &MotorSetpoint, motorKp, motorKi, motorKd, DIRECT);

// Variables
boolean pacing = false;
int decimalPlaces = 3;

// Function Declarations
void handleRoot();
void setESCMicroseconds(int microseconds);
double getDistanceTraveled();
void setServoAngle(int angle);
void calibrateQTR();
uint16_t getPosition();
double getRPS();
double calculateRPS();

void setup() {
    Serial.begin(115200);

    Wire.begin();
    as5600.resetCumulativePosition(0);
    ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(ESC_PIN, LED_CHANNEL);
    ledcSetup(SERVO_LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(SERVO_PIN, SERVO_LED_CHANNEL);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    setESCMicroseconds(1500);
    setServoAngle(90);

    Input = getPosition();
    Setpoint = 3500;
    steeringPID.SetOutputLimits(-90, 90);
    steeringPID.SetMode(AUTOMATIC);

    MotorInput = getRPS();
    MotorSetpoint = calculateRPS();
    motorPID.SetOutputLimits(0, 500);
    motorPID.SetMode(AUTOMATIC);

    WiFi.softAP(ssid, password);

    server.on("/", handleRoot);
    server.begin();

    Serial.println("HTTP server started");
    Serial.println("Connect to WiFi: " + String(ssid));
    Serial.println("Then go to: http://192.168.4.1");
}

void loop() {
    server.handleClient();

    if (pacing) {

        // Remove this line after testing
        // setESCMicroseconds(microsecondsTemporary);

        Input = getPosition();
        steeringPID.Compute();
        setServoAngle(90 + Output);

        // Motor Control
        MotorInput = getRPS();
        motorPID.Compute();
        setESCMicroseconds(1500 + MotorOutput);

        Serial.println(1500 + MotorOutput);

        if (getDistanceTraveled() >= distance) {
            pacing = false;
            setServoAngle(90);
            setESCMicroseconds(NEUTRAL_THROTTLE);
            as5600.resetCumulativePosition(0);
            Serial.println("Program Stopped");
        }
    }
    else {
        if (digitalRead(BUTTON_PIN) == LOW) {
            calibrateQTR();
        }
    }
}

void handleRoot() {
    if (server.method() == HTTP_POST) {
        if (server.hasArg("kp")) {
            Kp = server.arg("kp").toDouble();
        }
        if (server.hasArg("kd")) {
            Kd = server.arg("kd").toDouble();
        }
        if (server.hasArg("ki")) {
            Ki = server.arg("ki").toDouble();
        }
        if (server.hasArg("motorkp")) {
            motorKp = server.arg("motorkp").toDouble();
        }
        if (server.hasArg("motorkd")) {
            motorKd = server.arg("motorkd").toDouble();
        }
        if (server.hasArg("motorki")) {
            motorKi = server.arg("motorki").toDouble();
        }
        if (server.hasArg("distance")) {
            distance = server.arg("distance").toDouble();
        }
        if (server.hasArg("goalTime")) {
            goalTime = server.arg("goalTime").toDouble();
        }
        if (server.hasArg("microsecondsTemporary")) {
            microsecondsTemporary = server.arg("microsecondsTemporary").toInt();
        }

        as5600.resetCumulativePosition(0);
        steeringPID.SetTunings(Kp, Ki, Kd);
        motorPID.SetTunings(motorKp, motorKi, motorKd);
        MotorSetpoint = calculateRPS();
        pacing = true;
    }

    String html = "<!DOCTYPE html><html><body>";
    html += "<h1>Pacer Control Board</h1>";
    html += "<form action='/' method='POST'>";

    html += "<h2>Steering PID Values</h2>";
    html += "<label for='kp'>Kp (Proportional Gain): </label>";
    html += "<input type='number' id='kp' name='kp' step='any' value='" + String(Kp, decimalPlaces) + "'><br>";
    html += "<label for='kd'>Kd (Derivative Gain): </label>";
    html += "<input type='number' id='kd' name='kd' step='any' value='" + String(Kd, decimalPlaces) + "'><br>";
    html += "<label for='ki'>Ki (Integral Gain): </label>";
    html += "<input type='number' id='ki' name='ki' step='any' value='" + String(Ki, decimalPlaces) + "'><br>";

    html += "<h2>Motor PID Values</h2>";
    html += "<label for='motorkp'>Kp (Proportional Gain): </label>";
    html += "<input type='number' id='motorkp' name='motorkp' step='any' value='" + String(motorKp, decimalPlaces) + "'><br>";
    html += "<label for='motorkd'>Kd (Derivative Gain): </label>";
    html += "<input type='number' id='motorkd' name='motorkd' step='any' value='" + String(motorKd, decimalPlaces) + "'><br>";
    html += "<label for='motorki'>Ki (Integral Gain): </label>";
    html += "<input type='number' id='motorki' name='motorki' step='any' value='" + String(motorKi, decimalPlaces) + "'><br>";

    html += "<h2>Pacing Parameters</h2>";
    html += "<label for='distance'>Distance (Meters): </label>";
    html += "<input type='number' id='distance' name='distance' step='any' value='" + String(distance, decimalPlaces) + "'><br>";
    html += "<label for='goalTime'>Pace Time (Seconds): </label>";
    html += "<input type='number' id='goalTime' name='goalTime' step='any' value='" + String(goalTime, decimalPlaces) + "'><br>";

    html += "<h2>Temporary Motor Speed Control</h2>";
    html += "<label for='microsecondsTemporary'>Microseconds: </label>";
    html += "<input type='number' id='microsecondsTemporary' name='microsecondsTemporary' value='" + String(microsecondsTemporary) + "'><br>";
    
    html += "<input type='submit' value='Start'>";

    html += "</form>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void setESCMicroseconds(int microseconds) {
  ledcWrite(LED_CHANNEL, int((microseconds / 20000.0) * 65535.0));
}

double getDistanceTraveled() {
    double motorRotations = as5600.getCumulativePosition() / 4096.0;
    double wheelRotations = motorRotations / GEAR_RATIO;
    double traveled = wheelRotations * WHEEL_CIRCUMFERENCE;

    return -traveled;
}

void setServoAngle(int angle) {
    int pulse_us = map(angle, 0, 180, 1000, 2000);
    int duty = (pulse_us * 65535) / 20000;
    ledcWrite(SERVO_LED_CHANNEL, duty);
}

void calibrateQTR() {
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 13, 18, 19, 4, 12, 23}, SensorCount);
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

double getRPS() {
    return -((as5600.getAngularSpeed(AS5600_MODE_RPM) / 60.0) / GEAR_RATIO);
}

double calculateRPS() {
    if (goalTime == 0) {
        return 0.0;
    }
    return ((distance / goalTime) / WHEEL_CIRCUMFERENCE);
}