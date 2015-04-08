
#ifndef EncoderMM_h
#define EncoderMM_h

#include "Arduino.h"

class Encoder {
  private:
    byte chA, chB;
    int pos = 0;
    bool currState = LOW, lastState = LOW;    
  public:
    Encoder( byte chA , byte chB );
    
    byte getChA();
    byte getChB();
    int getPos();    
    bool getCurrState();
    bool getLastState();

    void setCurrState( bool currState );
    void setLastState( bool lastState );
    
    void readEnc();
    void printEnc(int pos);
};

#endif
