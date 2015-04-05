
#include <Motor.h>
#include <Encoder.h>
#include <Sensor.h>
#include <LED.h>


/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015. Hi
**/

#include <math.h>
#define NUMSENSORS 6
//Pin Assignments for IR Sensors
const byte leftEmitIR = 3 , leftRecIR = A12 , leftLED = 11;
const byte leftDiagEmitIR = 28 , leftDiagRecIR = A18 , leftDiagLED = 30;
const byte leftFrontEmitIR = 4 , leftFrontRecIR = A11 , leftFrontLED = 13;

const byte rightEmitIR = 2 , rightRecIR = A13 , rightLED = 12;
const byte rightDiagEmitIR = 25 , rightDiagRecIR = A15 , rightDiagLED = 27 ;
const byte rightFrontEmitIR = 5 , rightFrontRecIR = A10 , rightFrontLED = 14;

/********** LEFT MOTOR VARS **********/

//Left Enable            // Connection correlation to MM schematic
const byte L_Enable = A2;  // (H-Bridge pin 1)

//Motors
const byte L_Mtr_A = A6;  // (H-Bridge pin 2)  //1A
const byte L_Mtr_B = A7;  // (H-Bridge pin 7)  //2A

//Encoder vars
const byte L_CH_A = 9;          // (H-Bridge SV3 pin 6)
const byte L_CH_B = 10;         // (H-Bridge SV3 pin 5)

/********** RIGHT MOTOR VARS **********/

//Right Enable            // Connection correlation to MM schematic
const int R_Enable = A3;  // (H-Bridge pin ?)

//Motors
const int R_Mtr_A = A8;  // (H-Bridge pin ?)  //3A
const int R_Mtr_B = A9;  // (H-Bridge pin ?)  //4A

//Encoder vars
int R_CH_A = 7;          // (H-Bridge SV3 pin ?)
int R_CH_B = 8;         // (H-Bridge SV3 pin ?)


//SENSOR Threshold Values
unsigned int minThresh = 650;
unsigned int maxThresh = 700;
  
int leftTicks = 0;
int rightTicks = 0;


Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];


//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
int encoderTicks = 0;

unsigned long curt = 0; 

//Encoder Information
volatile int state = LOW;


void setup(){
  Serial.begin(9600); //Used for Debugging
  if(debugOn){
    Serial.println("->setup");
  }

  //Trash taken care of in classes
  /*
  pinMode(irLtOut,OUTPUT);       pinMode(irLtIn, INPUT), pinMode(irLtLED, OUTPUT);
  pinMode(irLtDiagOut,OUTPUT);   pinMode(irLtDiagIn, INPUT), pinMode(irLtDiagLED, OUTPUT);
  pinMode(irLtFrntOut,OUTPUT);   pinMode(irLtFrntIn, INPUT), pinMode(irLtFrntLED, OUTPUT);
  pinMode(irRtOut,OUTPUT);       pinMode(irRtIn, INPUT), pinMode(irRtLED, OUTPUT);
  pinMode(irRtDiagOut,OUTPUT);   pinMode(irRtDiagIn, INPUT), pinMode(irRtDiagLED, OUTPUT);
  pinMode(irRtFrntOut,OUTPUT);   pinMode(irRtFrntIn, INPUT), pinMode(irRtFrntLED, OUTPUT);
  */
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
  
  //Left left
  sensor[0] = new Sensor( leftEmitIR , leftRecIR , leftLED );
  //Left Diag
  sensor[1] = new Sensor( leftDiagEmitIR , leftDiagRecIR , leftDiagLED );
  //Left Forward
  sensor[2] = new Sensor( leftFrontEmitIR , leftDiagRecIR , leftDiagLED );
  //Right Forward
  sensor[3] = new Sensor( rightEmitIR , rightRecIR , rightLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right right
  sensor[5] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED ); 
  
  
  attachInterrupt(RchA, encoderTick, CHANGE);
  
  
  delay(7000);
  setBothMtrsForward();
}

void loop(){
  if(debugOn){
    Serial.println("->Loop"); //Used for Debugging 
  }
  PID();
}

/*
* Encoder Tick Functions
*
*
**/
void encoderTick(){
    encoderTicks++; 
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
  
  int left = getLeftIR();
  int right = getRightIR();
  int diff = 0;
  
  diff = left - right;
  
  if (diff >= 20){
    setBothMtrsForward(byte speed = 255);
    goStraight(distancePerMove, distancePerMove+1);
  }
  
  else if (diff < -20){
    goStraight(distancePerMove+1, distancePerMove);
  }
  else{
    goStraight(distancePerMove, distancePerMove);
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

void goStraight(int leftPWM, int rightPWM){
  setBothMtrsForward();
   if(debugOn){
     Serial.println("->goStraight"); //Used for Debugging 
  }
  mtrL->setSpeed(leftPWM);
  mtrR->setSpeed(rightPWM);
//  analogWrite(RpwmA, rightpwm);
//  analogWrite(LpwmA, leftpwm);
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

void readDasSensors(){
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }
  
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getIR();
  }
  /*
  getLeftIR();
  getLeftDiagIR();
  getLeftFrontIR();
  getRightIR();
  getRightDiagIR();
  getRightFrontIR();
  */
}

/*
* Smooths sensor data
* - increasing numReading increases accuracy, but increasing by 5 increases calculation time by ~500us
*/


//THE POWER OF CLASSES MUTHA FUCKAS
/*
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

double getIR(int emitPin, int recPin){
  if(debugOn){
    Serial.println("->getIR"); //Used for Debugging 
  }
 
  double senseVal = 0;
  curt = micros();
  sensor[0]->getLED().setHIGH();  
//  digitalWrite(pinOut,HIGH); //LED
  delayMicroseconds(80);
  senseVal = sensorData(sensor[0]->getRecPin());
  sensor[0]->getLED().setLOW();  
//  digitalWrite(pinOut,LOW); //LED
  return getCentiDistance(senseVal);
}

double getLeftIR(){
  if(debugOn){
    Serial.println("->getLeftIR"); //Used for Debugging 
  }
  return getIR( leftEmitIR , leftRecIR );
  
}

int getLeftDiagIR(){
  if(debugOn){
    Serial.println("->getLeftDiagIR"); //Used for Debugging 
  }
  return getIR( leftDiagEmitIR , leftDiagRecIR );
  
}

int getLeftFrontIR(){
  if(debugOn){
    Serial.println("->getLeftFrontIR"); //Used for Debugging 
  }
  return getIR( leftFrontEmitIR , leftFrontRecIR );
}

int getRightIR(){
  if(debugOn){
    Serial.println("->getRightIR"); //Used for Debugging 
  }
   return getIR( leftEmitIR , leftRecIR );
}

int getRightDiagIR(){
  if(debugOn){
    Serial.println("->getRightDiagIR"); //Used for Debugging 
  }
   return getIR( rightDiagEmitIR , rightDiagRecIR );
}

int getRightFrontIR(){
  if(debugOn){
    Serial.println("->getRightFrontIR"); //Used for Debugging 
  }
   return getIR( rightFrontEmitIR , rightFrontRecIR );
}
*/
/*
* GET Encoder Functions
*
*
**/


//These are trash
/*
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
*/

// MrSwirlyEye's functions (to rid of initialize.h

void readSensors() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->setSensorRead(  analogRead( sensor[i]->getRecPin() )  );
}

void printSensorReadings() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    Serial.print("Sensor#");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensor[i]->getSensorRead());    
    
    if ( i == (NUMSENSORS - 1) )
      Serial.println();
    else
      Serial.print(" // ");
  }     
}

void sensorsToLEDs() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    if ( sensor[i]->getSensorRead() < minThresh )
      sensor[i]->getLED().setLOW();
    else
      sensor[i]->getLED().setHIGH();
  }      
}

/********** MOTOR FUNCTIONS **********/

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


/********** ENCODER FUNCTIONS **********/

void readBothEnc() {
  mtrL->getEnc().readEnc();
  mtrR->getEnc().readEnc();
}
