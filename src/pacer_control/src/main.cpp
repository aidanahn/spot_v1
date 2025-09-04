#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "PacerESP32";
const char* password = "04082008";

WebServer server(80);

// Steering PID Constants
float Kp = 0.0;
float Kd = 0.0;
float Ki = 0.0;

// Function Declarations
void handleRoot();

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.softAP(ssid, password);

    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();

    Serial.println("HTTP server started");
    Serial.println("Connect to WiFi: " + String(ssid));
    Serial.println("Then go to: http://192.168.4.1");
}

void loop() {
    server.handleClient();
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
    }

    String html = "<!DOCTYPE html><html><body>";
    html += "<h1>Pacer Control Board</h1>";
    html += "<form action='/' method='POST'>";

    html += "<label for='kp'>Kp (Proportional Gain): </label>";
    html += "<input type='number' id='kp' name='kp' value='" + String(Kp) + "'><br>";
    html += "<label for='kd'>Kd (Derivative Gain): </label>";
    html += "<input type='number' id='kd' name='kd' value='" + String(Kd) + "'><br>";
    html += "<label for='ki'>Ki (Integral Gain): </label>";
    html += "<input type='number' id='ki' name='ki' value='" + String(Ki) + "'><br>";
    html += "<input type='submit' value='Start'>";

    html += "</form>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}