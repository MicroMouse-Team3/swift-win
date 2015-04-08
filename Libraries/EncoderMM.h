
#ifndef EncoderMM_h
#define EncoderMM_h

#include "Arduino.h"

class Encoder {
  private:
    byte chA, chB;
    int pos = 0;
    bool currState = LOW, lastState = LOW;    
  public:
    Encoder( byte , byte );
    
    byte getChA();
    byte getChB();
    int getPos();    
    bool getCurrState();
    bool getLastState();

    void setCurrState( bool );
    void setLastState( bool );
    
    void readEnc();
    void printEnc(int);
};

#endif
