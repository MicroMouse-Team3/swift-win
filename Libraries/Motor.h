
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "EncoderMM.h"

class Motor {
  private:
    byte enablePin, reversePin, forwardPin, speed = 200;
    Encoder enc;
  
  public:       
    Motor( byte enablePin , byte reversePin , byte forwardPin , byte enc_reversePin , byte enc_forwardPin );

    Encoder& getEnc();

    byte getEnablePin();
    
    void setSpeed( byte speed );
    void setForward( byte spdB);
    void setBackward( byte spdA);
};

#endif
