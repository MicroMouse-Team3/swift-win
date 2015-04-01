
#include "Arduino.h"
#include "Encoder.h"

Encoder::Encoder( byte chA , byte chB ) {
  pinMode( this->chA = chA , INPUT );
  pinMode( this->chB = chB , INPUT );
}
int Encoder::getPos() {
  return pos;
}

byte Encoder::getChA() {
  return chA;
}

byte Encoder::getChB() {
  return chB;
}

void Encoder::setCurrState(bool currState) {
  this->currState = currState;
}

bool Encoder::getCurrState() {
  return currState;
}

void Encoder::setLastState(bool lastState) {
  this->lastState = lastState;
}

bool Encoder::getLastState() {
  return lastState;
}

void Encoder::readEnc() {
  
  currState = digitalRead(chA);
  
  if ( (lastState == LOW) && (currState == HIGH) ) {
    if ( digitalRead(chB) == LOW )
      pos--;
    else
      pos++;

    printEnc(pos);        
  }
  lastState = currState;  
}

void Encoder::printEnc(int) {
    if ( pos > 0 )
      Serial.print("Left Mtr moving forward: ");
    else
      Serial.print("Left Mtr moving backward: ");
 
    Serial.print(pos);
    Serial.println();
}
