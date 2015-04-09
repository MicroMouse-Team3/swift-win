
#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor( byte sensEmitPin , byte sensRecPin , byte ledPin ) : led(ledPin) {
  pinMode( this->sensEmitPin = sensEmitPin , OUTPUT );
  pinMode( this->sensRecPin = sensRecPin , INPUT );
  pinMode( this->ledPin = ledPin , OUTPUT );
}

LED & Sensor::getLED() {
  return led;
}

void Sensor::setSensorRead( unsigned int recRead) {
  this->recRead = recRead; 
}

double Sensor::getSensorRead() {
  return recRead;
}

byte Sensor::getEmitPin() {
  return sensEmitPin;
}

byte Sensor::getRecPin() {
  return sensRecPin;
}



/***/

double Sensor::getIR(){
//  double senseVal = 0;
//  unsigned long curt = micros();
  led.setHIGH();
//  sensor[0]->getLED().setHIGH();  
  //digitalWrite( ledPin , HIGH ); //LED
  delayMicroseconds(80);
  recRead = sensorData( sensRecPin );
  led.setLOW();  
//  digitalWrite( ledPin , LOW ); //LED
  return getDistance(recRead);
}

double Sensor::sensorData( byte sensRecPin ){
  const byte numReads = 15;
  int readings[numReads] = {0,};
  byte i = 0;
  unsigned int total = 0;
  unsigned int average = 0;
  boolean running = true;
  /*
  for ( byte currRead = 0; currRead < numReads; currRead++)
    readings[currRead] = 0;   
    */
  while(running){
    total -= readings[i];
    readings[i] = analogRead(sensRecPin);
    total += readings[i++];
//    i++;
    if( i >= numReads ){
      average = total / numReads;
      running = false;
    }
  }
  return average;
}

/* Nonlinear Regression.very inefficient. This polynomial only works within a specific range. Must find better way to find the correct Y*/
double Sensor::getDistance( double recRead ){
  double result = 0;
  double finalResult = 0;
  boolean matchFound = false;
  for( double count = 0.0 ; count < 23.0 ; count += .1 ){
    // will be changed to calibrate
     result = -0.000472391*pow(count,6) + 0.040086828*pow(count,5) - 1.331327*pow(count,4) + 21.57032*pow(count,3) - 168.49*pow(count,2) - 455.6145*count + 721.3995;
    
     if( result < recRead + 10 && result > recRead - 10){
        matchFound = true;
        finalResult = count;       
      }
  }
  if(matchFound)
    return finalResult;
  else
    return 0.0;
}