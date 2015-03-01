#include <math.h>
#include <stdio.h>

#define KP 1
#define KI 1
#define KD 1
#define SETPOINTU 1024
#define SETPOINTK 512

#define LEFTENCPIN 0
#define RIGHTENCPIN 1
#define LEFTPWMPIN 2
#define RIGHTPWMPIN 3

//prototype
int PID(int, int, int&, unsigned long&);

void setup(){
  
  
    pinMode(LEFTENCPIN, INPUT);
    pinMode(RIGHTENCPIN, INPUT);
    
    pinMode(LEFTPWMPIN, OUTPUT);
    pinMode(RIGHTPWMPIN, OUTPUT);
    
    
  
}

void loop() {
  
  int LEncoder=0, REncoder=0, setpoint=0;
  int lastRightError, lastLeftError, rightOutput, leftOutput;
  unsigned long lastRightTime= 0, lastLeftTime= 0;
  
  //saving for later
  //int rightEncoder=0, leftEncoder=0;
/*  while(LEncoder <= setpoint){
     REncoder = digitalRead(rightEndocer);
     LEncoder = digitalRead(leftEncoder);
     analogWrite (LMotor, LPWM);
     analogWrite (RMotor, RPWM);
  }*/
  
   rightOutput= PID(REncoder, setpoint, lastRightError, lastRightTime);
   leftOutput = PID(LEncoder, setpoint, lastLeftError, lastLeftTime);
   

}

int PID(int encoder, int setpoint, int &lastError, unsigned long &lastTime) {
   //Setting up variables
   int error;
   int output=0;
   int integral =0, proportion=0, derivative=0;
   
   //fiding the time and error
   unsigned long currentTime = millis();
   unsigned long deltaTime = currentTime - lastTime;
   error = encoder - setpoint;
   
   //Calculating the output
   proportion = error * KP;
   integral += error*deltaTime;
   integral *= KI;   
   derivative = ((error-lastError)/deltaTime)*KD;   
   output = proportion + integral + derivative;
   
   //saving current errors into last (passed in by reference)
   lastError = error;
   lastTime = currentTime;
   
   return output;
}



