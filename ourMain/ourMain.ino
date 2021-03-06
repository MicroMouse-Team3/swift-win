#include <Motor.h>
#include <Encoder.h>
#include <LED.h>
#include <math.h>

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

//Global Values
const int distancePerMove = 30;
byte speed = 50;
unsigned long curt = 0; 
int leftEnc = 0;
int rightEnc = 0;

//Colins Variables
Motor * mtrL;
Motor * mtrR;


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
  
  //Cdawg
  
  //Left Enable            // Connection correlation to MM schematic
  const byte L_Enable = 3;  // (H-Bridge pin 1)

  //Motors
  const byte L_Mtr_A = 6;  // (H-Bridge pin 2)  //1A
  const byte L_Mtr_B = 9;  // (H-Bridge pin 7)  //2A

  //Encoder vars
  const byte L_CH_A = 4;          // (H-Bridge SV3 pin 6)
  const byte L_CH_B = 13;         // (H-Bridge SV3 pin 5)

  
  const int R_Enable = 3;  // (H-Bridge pin ?)

  //Motors
  const int R_Mtr_A = 11;  // (H-Bridge pin ?)  //3A
  const int R_Mtr_B = 5;  // (H-Bridge pin ?)  //4A


  //Encoder vars
  int R_CH_A = 4;          // (H-Bridge SV3 pin ?)
  int R_CH_B = 13;         // (H-Bridge SV3 pin ?)
  
  attachInterrupt(RchA, 
  
  
   mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
   mtrR = new Motor( R_Enable , R_Mtr_A , L_Mtr_B , R_CH_A , R_CH_B );
   
   delay(5000);
   
   setBothMtrsForward();
   setBothMtrsSpd(speed);
}

void loop(){
  if(debugOn){
    Serial.println("->Loop"); //Used for Debugging 
  }
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
  setBothMtrsForward();
  
  
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

double getIR(int pinOut, int pinIn){
  if(debugOn){
    Serial.println("->getIR"); //Used for Debugging 
  }
 
  double senseVal = 0;
  curt = micros();
  digitalWrite(pinOut,HIGH);
  delayMicroseconds(80);
  senseVal = sensorData(pinIn);
  digitalWrite(pinOut,LOW);
  return getCentiDistance(senseVal);
}

void readSensors(){
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }
  getLeftIR();
  getLeftDiagIR();
  getLeftFrontIR();
  getRightIR();
  getRightDiagIR();
  getRightFrontIR();
}

/*
* Smooths sensor data
* - increasing numReading increases accuracy, but increasing by 5 increases calculation time by ~500us
*/

double sensorData(int readPin){
  int numReadings = 15;
  int readings[numReadings];
  int index = 0;
  int total = 0;
  int average = 0;
  boolean run = true;
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;   
  while(run){
    total = total -readings[index];
    readings[index] = analogRead(readPin);
    total = total + readings[index];
    index++;
    if(index >= numReadings){
      average = total/numReadings;
      run = false;
    }
  }
  return average;
}

/* Nonlinear Regression.very inefficient. This polynomial only works within a specific range. Must find better way to find the correct Y*/
double getCentiDistance(double irSensorValue){
  double result = 0;
  double finalResult = 0;
  boolean matchFound = false;
  for(double count = 0.0; count < 23.0; count += .1){
    // will be changed to calibrate
     result = 0.00022738*pow(count,6)- 0.0169745*pow(count,5) + 0.470274283*pow(count,4) - 5.840060615*pow(count,3) + 31.350111*pow(count,2) - 97.704*count + 716.594;
    
     if(result < irSensorValue + 10 && result > irSensorValue - 10){
        matchFound = true;
        finalResult = count;
       
      }
  }
  if(matchFound)
    return finalResult;
    else
    return 0.0;
}

double getLeftIR(){
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

void addLeftEncoder(){
  leftEnc++;
}

void addRightEncoder(){
  rightEnc++;
}

///////////C DAWG CODE BELOW/////////////////////
void setLMtrSpd( byte speed ) {
  mtrL->setSpeed(speed);     
}

void setRMtrSpd( byte speed ) {
  mtrR->setSpeed(speed);
}

void setBothMtrsSpd( byte speed ) {
  mtrL->setSpeed(speed);
  mtrR->setSpeed(speed);
}

void setBothMtrsForward() {
  mtrL->setForward();
  mtrR->setForward();
}

void setBothMtrsBackward() {
  mtrL->setBackward();
  mtrR->setBackward();
}

void setMtrsLeftTurn() {
  mtrL->setBackward();
  mtrR->setForward();
}

void setMtrsRightTurn() {
  mtrL->setForward();
  mtrR->setBackward();
} 

