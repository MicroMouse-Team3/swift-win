
#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor( byte sensorPin, byte ledPin ) : led(ledPin) {
  this->sensorPin = sensorPin;
}

LED& Sensor::getLED() {
  return led;
}

void Sensor::setSensorReading( unsigned int sensorReading) {
  this->sensorReading = sensorReading; 
}

unsigned int Sensor::getSensorReading() {
  return sensorReading;
}

byte Sensor::getSensorPin() {
  return sensorPin;
}
