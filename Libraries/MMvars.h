
#ifndef MMvars_h
#define MMvars_h

#include "Arduino.h"

bool wallLeft;
bool wallRight;
bool wallFront;

boolean frontWall;
boolean leftWall;
boolean leftFrontWall;
boolean rightWall;
boolean rightFrontWall;
boolean myBool;
boolean yourBool;


const byte NUMSENSORS = 6;

Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];

void setLEDsON() {  
  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->getLED().setHIGH();
}

void setLEDsOFF() {  
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) 
    sensor[i]->getLED().setLOW();
}

void moveForward( byte speed ) {
  mtrL->setForward( speed );
  mtrR->setForward( speed );
}

void moveRight( byte speed ) {
  mtrL->setForward( speed );
  mtrR->setBackward( speed );
}

void moveLeft( byte speed ) {
  mtrL->setBackward( speed );
  mtrR->setForward( speed );
}

void moveBackward( byte speed ) {
  mtrL->setBackward( speed );
  mtrR->setBackward( speed );
}

void checkForWalls() {
  wallLeft = sensor[0]->getIR() > 600;
  wallFront = sensor[2]->getIR() > 200;
  wallRight = sensor[5]->getIR() > 600; 
}

void initializePins() {
    pinMode(0, INPUT);
  pinMode(1, OUTPUT);
  pinMode(rightEmitIR, OUTPUT);
  pinMode(leftEmitIR, OUTPUT);
  pinMode(leftFrontEmitIR, OUTPUT);
  pinMode(rightFrontEmitIR, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(R_CH_A, INPUT);
  pinMode(R_CH_B, INPUT);
  pinMode(L_CH_A, INPUT);
  pinMode(L_CH_B, INPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(leftFrontLED, OUTPUT);
  pinMode(rightFrontLED, OUTPUT);
  pinMode(15, INPUT);
  pinMode(L_Enable, OUTPUT);
  pinMode(R_Enable, OUTPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(L_Mtr_A, OUTPUT);
  pinMode(L_Mtr_B, OUTPUT);
  pinMode(R_Mtr_A, OUTPUT);
  pinMode(R_Mtr_B, OUTPUT);
  pinMode(24, INPUT);
  pinMode(rightDiagEmitIR, OUTPUT);
  pinMode(26, INPUT);
  pinMode(rightDiagLED, OUTPUT);
  pinMode(leftDiagEmitIR, OUTPUT);
  pinMode(29, INPUT);
  pinMode(leftDiagLED, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
}

void initializeMotors() {
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
}

void initializeSensors() {
  //Left left
  sensor[0] = new Sensor( leftEmitIR , leftRecIR , leftLED );
  //Left Diag
  sensor[1] = new Sensor( leftDiagEmitIR , leftDiagRecIR , leftDiagLED );
  //Left Front
  sensor[2] = new Sensor( leftFrontEmitIR , leftFrontRecIR , leftFrontLED );
  //Right Front
  sensor[3] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right Right
  sensor[5] = new Sensor( rightEmitIR , rightRecIR , rightLED );
}



void ledTest() {
    setLEDsON();  
  delay(2000);
  setLEDsOFF();  
  delay(2000);
}

/********** ENCODER FUNCTIONS **********/

void readBothEnc() {
  mtrL->getEnc().readEnc();
  mtrR->getEnc().readEnc();
}


  
#endif
