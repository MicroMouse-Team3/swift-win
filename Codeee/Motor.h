
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Encoder.h"

class Motor {
  private:
    byte enablePin, chA, chB, speed;
    Encoder enc;
  
  public:       
    Motor( byte enablePin , byte mtr_chA , byte mtr_chB , byte enc_chA , byte enc_chB );

    Encoder& getEnc();

    byte getEnablePin();
    
    void setSpeed( byte );
    void setForward();
    void setBackward();
};

#endif
