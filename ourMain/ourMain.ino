/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015
**/

//Pin Assignments for IR Sensors
const int irLtOut = 3, irLtIn = A12, irLtLED = 11;
const int irLtDiagOut = 28,irLtDiagIn = A18, irLtDiagLED = 30;
const int irLtFrntOut = 4,irLtFrntIn = A11, irLtFrntLED = 13;
const int irRtOut = 2, irRtIn = A13, irRtLED = 12;
const int irRtDiagOut = 25, irRtDiagIn = A15, irRtDiagLED = 27 ;
const int irRtFrntOut = 5, irRtFrntIn = A10, irRtFrntLED = 14;

//Pin Assignments for Encoders
const int RchA = 7, RchB = 8;
const int LchA = 9, LchB = 10;

//Pin Assignments for Motors
const int LpwmA = A6, LpwmB = A7;
const int RpwmA = A8, RpwmB = A9;

//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

unsigned long curt = 0; 

void setup(){
  Serial.begin(9600); //Used for Debugging
  if(debugOn){
    Serial.println("->setup");
  }
  
  pinMode(irLtOut,OUTPUT);       pinMode(irLtIn, INPUT), pinMode(irLtLED, OUTPUT);
  pinMode(irLtDiagOut,OUTPUT);   pinMode(irLtDiagIn, INPUT), pinMode(irLtDiagLED, OUTPUT);
  pinMode(irLtFrntOut,OUTPUT);   pinMode(irLtFrntIn, INPUT), pinMode(irLtFrntLED, OUTPUT);
  pinMode(irRtOut,OUTPUT);       pinMode(irRtIn, INPUT), pinMode(irRtLED, OUTPUT);
  pinMode(irRtDiagOut,OUTPUT);   pinMode(irRtDiagIn, INPUT), pinMode(irRtDiagLED, OUTPUT);
  pinMode(irRtFrntOut,OUTPUT);   pinMode(irRtFrntIn, INPUT), pinMode(irRtFrntLED, OUTPUT);
  
}

void loop(){
  if(debugOn){
    Serial.println("->Loop"); //Used for Debugging 
  }
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
  if(debugOn){
    Serial.println("->PID"); //Used for Debugging 
  }
  
}

/*
* Driving Functions
*
*
**/

void drive(){
  if(debugOn){
    Serial.println("->Drive"); //Used for Debugging 
  }
  
}

void turnLeft(){
  if(debugOn){
    Serial.println("->turnLeft"); //Used for Debugging 
  }
}

void turnRight(){
  if(debugOn){
   Serial.println("->turnRight"); //Used for Debugging 
  }  
}

void goStraight(){
   if(debugOn){
     Serial.println("->goStraight"); //Used for Debugging 
  }
  
}

void turnAround(){
  if(debugOn){
    Serial.println("->turnAround"); //Used for Debugging 
  }
  
}

int distanceTraveled(){
  if(debugOn){
    Serial.println("->distanceTraveled"); //Used for Debugging 
  }
}

void resetDistanceTraveled(){
  if(debugOn){
    Serial.println("->resetDistanceTraveled"); //Used for Debugging 
  }
}

/*
* GET IR Functions
*
*
**/

int getIR(int pinOut, int pinIn){
  if(debugOn){
    Serial.println("->getIR"); //Used for Debugging 
  }
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
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }
  getLeftIR();
  delay(10); // must find out how long for other pulses to dissipate, improve accuracy
  getLeftDiagIR();
  getLeftFrontIR();
  getRightIR();
  getRightDiagIR();
  getRightFrontIR();
}

int getLeftIR(){
  if(debugOn){
    Serial.println("->getLeftIR"); //Used for Debugging 
  }
  return getIR(irLtOut,irLtIn);
  
}

int getLeftDiagIR(){
  if(debugOn){
    Serial.println("->getLeftDiagIR"); //Used for Debugging 
  }
  return getIR(irLtDiagOut,irLtDiagIn);
  
}

int getLeftFrontIR(){
  if(debugOn){
    Serial.println("->getLeftFrontIR"); //Used for Debugging 
  }
  return getIR(irLtFrntOut,irLtFrntIn);
}

int getRightIR(){
  if(debugOn){
    Serial.println("->getRightIR"); //Used for Debugging 
  }
   return getIR(irLtOut,irLtIn);
}

int getRightDiagIR(){
  if(debugOn){
    Serial.println("->getRightDiagIR"); //Used for Debugging 
  }
   return getIR(irRtDiagOut,irRtDiagIn);
}

int getRightFrontIR(){
  if(debugOn){
    Serial.println("->getRightFrontIR"); //Used for Debugging 
  }
   return getIR(irRtFrntOut,irRtFrntIn);
}

/*
* GET Encoder Functions
*
*
**/

boolean tick(){
  if(debugOn){
    Serial.println("->tick"); //Used for Debugging 
  }
  
}

int getLeftEncod(){
  if(debugOn){
    Serial.println("->getLeftEncod"); //Used for Debugging 
  }
  
}

int getRightEncod(){
  if(debugOn){
    Serial.println("->getRightEncod"); //Used for Debugging 
  }
}


