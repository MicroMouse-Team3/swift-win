
#include "Arduino.h"
#include "Motor.h"

Motor::Motor( byte enablePin , byte chA , byte chB , byte enc_chA , byte enc_chB ) : enc( enc_chA , enc_chB) {
  pinMode( this->enablePin = enablePin , OUTPUT );
  pinMode( this->chA = chA , OUTPUT );
  pinMode( this->chB = chB , OUTPUT );
}

void Motor::setSpeed( byte speed ) {
  analogWrite( enablePin , this->speed = speed );
}

void Motor::setForward( byte spdA , byte spdB ) {
/*
  digitalWrite( chA , HIGH );
  digitalWrite( chB , LOW );
*/
  analogWrite( chA , spdA );
  analogWrite( chB , spdB );
}

void Motor::setBackward( byte spdA , byte spdB ) {
/*
  digitalWrite( chA , LOW );
  digitalWrite( chB , HIGH );
*/
  analogWrite( chA , spdA );
  analogWrite( chB , spdB );
}

byte Motor::getEnablePin() {
  return enablePin;
}

Encoder& Motor::getEnc() {
  return enc;
}  
