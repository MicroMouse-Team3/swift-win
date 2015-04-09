
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "LED.h"
#include <math.h>

class Sensor {
  private:
    double recRead;
    byte sensEmitPin, sensRecPin;
    LED led;
    
  public:
  byte ledPin;
    Sensor( byte sensEmitPin , byte sensRecPin , byte ledPin ); 
    
    LED& getLED();
    byte getEmitPin();
    byte getRecPin();
    double getSensorRead();
    double getIR();    
    double sensorData( byte sensRecPin );
    double getDistance( double recRead );
    
    void setSensorRead( unsigned int recRead );
    
    void readSensor();    
};

#endif
