
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
void Motor::setForward( byte forwardSpeedIn) {
/*
  digitalWrite( reversePin , HIGH );
  digitalWrite( forwardPin , LOW );
*/
  analogWrite( reversePin , 0 );
  analogWrite( forwardPin , forwardSpeedIn );
}
              //150   0
void Motor::setBackward(byte reverseSpeedIn ) {
/*
  digitalWrite( reversePin , LOW );
  digitalWrite( forwardPin , HIGH );
*/
  analogWrite( reversePin , reverseSpeedIn );
  analogWrite( forwardPin , 0 );
}

byte Motor::getEnablePin() {
  return enablePin;
}

Encoder& Motor::getEnc() {
  return enc;
}  
