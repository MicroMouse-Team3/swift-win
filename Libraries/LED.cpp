
#include "Arduino.h"
#include "LED.h"

LED::LED ( byte ledPin ) {
  pinMode ( this->ledPin = ledPin , OUTPUT );
}
    
void LED::setLOW() {
  digitalWrite ( ledPin , this->isON = LOW );
}

void LED::setHIGH() {
  digitalWrite ( ledPin , this->isON = HIGH );
}
