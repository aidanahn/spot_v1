#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 13, 18, 19, 21, 22, 23}, SensorCount);
  qtr.setEmitterPin(2);

  Serial.begin(9600);
}

void loop() {
  // read raw sensor values
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
}
