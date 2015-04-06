
#ifndef LED_h
#define LED_h

#include "Arduino.h"

class LED {
  private:
    byte ledPin;
    bool isON;
    
  public:
    LED( byte );
    
    void setLOW();
    void setHIGH();
};
  
#endif
