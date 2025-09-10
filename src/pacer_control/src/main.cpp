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
int microsecondsTemporary = NEUTRAL_THROTTLE;

// Steering PID Values
float Kp = 0.5;
float Kd = 0.0;
float Ki = 0.0;

// PID Variables
double Setpoint, Input, Output;
PID steeringPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables
boolean pacing = false;

// Function Declarations
void handleRoot();
void setESCMicroseconds(int microseconds);
double getDistanceTraveled();
void setServoAngle(int angle);
void calibrateQTR();
uint16_t getPosition();

void setup() {
    Serial.begin(115200);

    calibrateQTR();

    Wire.begin();
    as5600.resetCumulativePosition(0);
    ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(ESC_PIN, LED_CHANNEL);
    ledcSetup(SERVO_LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(SERVO_PIN, SERVO_LED_CHANNEL);
    setESCMicroseconds(1500);
    setServoAngle(90);

    Input = getPosition();
    Setpoint = 3500;
    steeringPID.SetOutputLimits(-90, 90);
    steeringPID.SetMode(AUTOMATIC);

    WiFi.softAP(ssid, password);

    server.on("/", handleRoot);
    server.begin();

    Serial.println("HTTP server started");
    Serial.println("Connect to WiFi: " + String(ssid));
    Serial.println("Then go to: http://192.168.4.1");
}

void loop() {
    server.handleClient();

    Input = getPosition();
    steeringPID.Compute();
    setServoAngle(90 + Output);

    if (pacing) {
        Serial.println(getDistanceTraveled());
        setESCMicroseconds(microsecondsTemporary);
        if (getDistanceTraveled() >= distance) {
            pacing = false;
            setESCMicroseconds(NEUTRAL_THROTTLE);
            as5600.resetCumulativePosition(0);
            Serial.println("Program Stopped");
        }
    }
}

void handleRoot() {
    if (server.method() == HTTP_POST) {
        if (server.hasArg("kp")) {
            Kp = server.arg("kp").toFloat();
        }
        if (server.hasArg("kd")) {
            Kd = server.arg("kd").toFloat();
        }
        if (server.hasArg("ki")) {
            Ki = server.arg("ki").toFloat();
        }
        if (server.hasArg("distance")) {
            distance = server.arg("distance").toFloat();
        }
        if (server.hasArg("microsecondsTemporary")) {
            microsecondsTemporary = server.arg("microsecondsTemporary").toInt();
        }

        as5600.resetCumulativePosition(0);
        pacing = true;
    }

    String html = "<!DOCTYPE html><html><body>";
    html += "<h1>Pacer Control Board</h1>";
    html += "<form action='/' method='POST'>";

    html += "<label for='kp'>Kp (Proportional Gain): </label>";
    html += "<input type='number' id='kp' name='kp' step='any' value='" + String(Kp) + "'><br>";
    html += "<label for='kd'>Kd (Derivative Gain): </label>";
    html += "<input type='number' id='kd' name='kd' step='any' value='" + String(Kd) + "'><br>";
    html += "<label for='ki'>Ki (Integral Gain): </label>";
    html += "<input type='number' id='ki' name='ki' step='any' value='" + String(Ki) + "'><br>";

    html += "<label for='distance'>Distance (Meters): </label>";
    html += "<input type='number' id='distance' name='distance' step='any' value='" + String(distance) + "'><br>";
    html += "<label for='goalTime'>Microseconds: </label>";
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
  uint16_t position = qtr.readLineBlack(sensorValues);
  return position;
}