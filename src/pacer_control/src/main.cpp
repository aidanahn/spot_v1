#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AS5600.h>

const char* ssid = "PacerESP32";
const char* password = "04082008";

WebServer server(80);

// AS5600 Encoder
AS5600 as5600;

// Pin Definitions
#define ESC_PIN 33

// Motor Constants
const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int LED_CHANNEL = 0;
const int NEUTRAL_THROTTLE = 1500;

// Distance Calculation Constants
const double GEAR_RATIO = (40.0 / 20.0) * (38.0 / 13.0);
const double WHEEL_DIAMETER = 121.9 / 1000.0;
const double WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Website Inputs
int microseconds = 1500;
double distance = 0.0;

// Steering PID Constants
float Kp = 0.0;
float Kd = 0.0;
float Ki = 0.0;

// Variables
boolean pacing = false;

// Function Declarations
void handleRoot();
void setESCMicroseconds(int microseconds);
double getDistanceTraveled();

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.softAP(ssid, password);

    server.on("/", handleRoot);
    server.begin();

    Serial.println("HTTP server started");
    Serial.println("Connect to WiFi: " + String(ssid));
    Serial.println("Then go to: http://192.168.4.1");

    Wire.begin();

    as5600.resetCumulativePosition(0);
    ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(ESC_PIN, LED_CHANNEL);
    setESCMicroseconds(1500);
}

void loop() {
    server.handleClient();

    if (pacing) {
        Serial.println(getDistanceTraveled());
        setESCMicroseconds(microseconds);
        if (getDistanceTraveled() >= distance) {
            setESCMicroseconds(NEUTRAL_THROTTLE);
            pacing = false;
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
        if (server.hasArg("microseconds")) {
            microseconds = server.arg("microseconds").toInt();
        }
        if (server.hasArg("distance")) {
            distance = server.arg("distance").toFloat();
        }

        pacing = true;
        as5600.resetCumulativePosition(0);
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

    html += "<label for='ki'>Microseconds: </label>";
    html += "<input type='number' id='microseconds' name='microseconds' step='any' value='" + String(microseconds) + "'><br>";
    html += "<label for='ki'>Distance (meters): </label>";
    html += "<input type='number' id='distance' name='distance' step='any' value='" + String(distance) + "'><br>";
    
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
