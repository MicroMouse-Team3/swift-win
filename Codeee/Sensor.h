
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "LED.h"

class Sensor {
  private:
    unsigned int sensorReading;
    byte sensorPin;
    LED led;
    
  public:
    Sensor( byte , byte ); 
    
    LED& getLED();
    byte getSensorPin();
    unsigned int getSensorReading();
    
    void setSensorReading( unsigned int );
    
    void readSensor();    
};

#endif
