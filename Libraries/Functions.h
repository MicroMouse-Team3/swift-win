
#ifndef Functions_h
#define Functions_h

#include "Arduino.h"
#include "Motor.h"
#include "EncoderMM.h"

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



int leftTicks = 0;
int rightTicks = 0;

//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
int encTickL = 0, encTickR = 0;

unsigned long curt = 0; 

//Encoder Information
volatile int state = LOW;


Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];

//SENSOR Threshold Values
unsigned int minThresh = 650;
unsigned int maxThresh = 700;

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

/*
* Encoder Tick Functions
*
*
**/
void incEncoderL() {
  encTickL++; 
}
void incEncoderR() {
  encTickR++;
}

#endif