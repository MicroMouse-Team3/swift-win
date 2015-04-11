
#include "Arduino.h"
#include "Motor.h"

Motor::Motor( byte enablePin , byte reversePin , byte forwardPin , byte enc_reversePin , byte enc_forwardPin ) : enc( enc_reversePin , enc_forwardPin) {
  this->enablePin = enablePin;
  this->reversePin = reversePin;
  this->forwardPin = forwardPin;
  speed = 200;  
}

void Motor::setSpeed( byte speed ) {
  //analogWrite( enablePin , this->speed = speed );
  digitalWrite( enablePin , this->speed = speed );
}
            //   150      0
void Motor::setForward( byte forwardSpeed , byte reverseSpeed) {
/*
  digitalWrite( reversePin , HIGH );
  digitalWrite( forwardPin , LOW );
*/
  analogWrite( reversePin , reverseSpeed );
  analogWrite( forwardPin , forwardSpeed );
}
              //150   0
void Motor::setBackward( byte spdA , byte spdB ) {
/*
  digitalWrite( reversePin , LOW );
  digitalWrite( forwardPin , HIGH );
*/
  analogWrite( reversePin , spdA );
  analogWrite( forwardPin , spdB );
}

byte Motor::getEnablePin() {
  return enablePin;
}

Encoder& Motor::getEnc() {
  return enc;
}  
