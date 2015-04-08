
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "EncoderMM.h"

class Motor {
  private:
    byte enablePin, chA, chB, speed = 200;
    Encoder enc;
  
  public:       
    Motor( byte enablePin , byte mtr_chA , byte mtr_chB , byte enc_chA , byte enc_chB );

    Encoder& getEnc();

    byte getEnablePin();
    
    void setSpeed( byte speed );
    void setForward( byte spdA , byte spdB );
    void setBackward( byte spdA , byte spdB );
};

#endif
