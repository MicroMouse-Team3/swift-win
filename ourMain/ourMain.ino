/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015
**/

const int irLtOut = 0, irLtIn =0;
const int irLtDiagOut =0,irLtDiagIn =0;
const int irLtFrntOut = 0,irLtFrntIn = 0;  // need pins
const int irRtOut =0, irRtIn = 0;
const int irRtDiagOut = 0, irRtDiagIn = 0;
const int irRtFrntOut = 0, irRtFrntIn = 0;

unsigned long curt = 0; 

void setup(){
  
  pinMode(irLtOut,OUTPUT);       pinMode(irLtIn, INPUT);
  pinMode(irLtDiagOut,OUTPUT);   pinMode(irLtDiagIn, INPUT);
  pinMode(irLtFrntOut,OUTPUT);   pinMode(irLtFrntIn, INPUT);
  pinMode(irRtOut,OUTPUT);       pinMode(irRtIn, INPUT);
  pinMode(irRtDiagOut,OUTPUT);   pinMode(irRtDiagIn, INPUT);
  pinMode(irRtFrntOut,OUTPUT);   pinMode(irRtFrntIn, INPUT);
  Serial.begin(9600);
}

void loop(){
  readSensors();
  drive();
  PID();
}

/*
* PID Functions
*
*
**/
void PID(){
  
}

/*
* Driving Functions
*
*
**/

void drive(){
  
}

void turnLeft(){
  
}

void turnRight(){
  
}

void goStraight(){
  
}

void turnAround(){
  
}

int distanceTraveled(){
  
}

void resetDistanceTraveled(){
  
}

/*
* GET IR Functions
*
*
**/

int getIR(int pinOut, int pinIn){
  int senseVal = 0;
  curt = micros();
  digitalWrite(pinOut,HIGH);
  while(micros()-curt <100);
  senseVal = analogRead(pinIn);
  digitalWrite(pinOut,LOW);
  delay(1000); // just for testing
  return senseVal;
}

void readSensors(){
  getLeftIR();
  delay(10); // must find out how long for other pulses to dissipate, improve accuracy
  getLeftDiagIR();
  getLeftFrontIR();
  getRightIR();
  getRightDiagIR();
  getRightFrontIR();
}

int getLeftIR(){
  return getIR(irLtOut,irLtIn);
  
}

int getLeftDiagIR(){
  return getIR(irLtDiagOut,irLtDiagIn);
  
}

int getLeftFrontIR(){
  return getIR(irLtFrntOut,irLtFrntIn);
}

int getRightIR(){
   return getIR(irLtOut,irLtIn);
}

int getRightDiagIR(){
   return getIR(irRtDiagOut,irRtDiagIn);
}

int getRightFrontIR(){
   return getIR(irRtFrntOut,irRtFrntIn);
}

/*
* GET Encoder Functions
*
*
**/

boolean tick(){
  
}

int getLeftEncod(){
  
}

int getRightEncod(){
  
}

 /*
   Need help fixing this
C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino/main.cpp:40: undefined reference to `setup'
C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino/main.cpp:43: undefined reference to `loop'
 */


