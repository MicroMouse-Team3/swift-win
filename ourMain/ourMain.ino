/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015
**/

//Pin Assignments
const int irLtOut = 0, irLtIn =0;
const int irLtDiagOut =0,irLtDiagIn =0;
const int irLtFrntOut = 0,irLtFrntIn = 0;  // need pins
const int irRtOut =0, irRtIn = 0;
const int irRtDiagOut = 0, irRtDiagIn = 0;
const int irRtFrntOut = 0, irRtFrntIn = 0;

//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

unsigned long curt = 0; 

void setup(){
  Serial.begin(9600); //Used for Debugging
  if(debugOn){
    Serial.println("->setup");
  }
  
  pinMode(irLtOut,OUTPUT);       pinMode(irLtIn, INPUT);
  pinMode(irLtDiagOut,OUTPUT);   pinMode(irLtDiagIn, INPUT);
  pinMode(irLtFrntOut,OUTPUT);   pinMode(irLtFrntIn, INPUT);
  pinMode(irRtOut,OUTPUT);       pinMode(irRtIn, INPUT);
  pinMode(irRtDiagOut,OUTPUT);   pinMode(irRtDiagIn, INPUT);
  pinMode(irRtFrntOut,OUTPUT);   pinMode(irRtFrntIn, INPUT);
  
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


