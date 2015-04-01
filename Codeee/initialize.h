
#include "Arduino.h"
#include "Motor.h"
#include "Sensor.h"

#define NUMSENSORS 3
#define NUMLEDS 3
/********** LEFT MOTOR VARS **********/

//Left Enable            // Connection correlation to MM schematic
const byte L_Enable = 3;  // (H-Bridge pin 1)

//Motors
const byte L_Mtr_A = 6;  // (H-Bridge pin 2)  //1A
const byte L_Mtr_B = 9;  // (H-Bridge pin 7)  //2A

//Encoder vars
const byte L_CH_A = 4;          // (H-Bridge SV3 pin 6)
const byte L_CH_B = 13;         // (H-Bridge SV3 pin 5)

/********** RIGHT MOTOR VARS **********/



//Right Enable            // Connection correlation to MM schematic
const int R_Enable = 3;  // (H-Bridge pin ?)

//Motors
const int R_Mtr_A = 11;  // (H-Bridge pin ?)  //3A
const int R_Mtr_B = 5;  // (H-Bridge pin ?)  //4A


//Encoder vars
int R_CH_A = 4;          // (H-Bridge SV3 pin ?)
int R_CH_B = 13;         // (H-Bridge SV3 pin ?)


//Set the speed/duty-cycle?
byte speed = 50;  //PWM? or uhh Duty Cycle?

const byte LeftSensor = A0;
const byte FrontSensor = A1;
const byte RightSensor = A2;
  
const byte leftLED = 8;
const byte frontLED = 12;
const byte rightLED = 7;
  
//SENSOR Threshold Values
unsigned int minThresh = 650;
unsigned int maxThresh = 700;
  
int leftTicks = 0;
int rightTicks = 0;


Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];

/********** SENSOR FUNCTIONS **********/

void readSensors() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->setSensorReading(  analogRead(sensor[i]->getSensorPin())  );
}

void printSensorReadings() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    Serial.print("Sensor#");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensor[i]->getSensorReading());    
    
    if ( i == (NUMSENSORS - 1) )
      Serial.println();
    else
      Serial.print(" // ");
  }     
}

void sensorsToLEDs() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    if ( sensor[i]->getSensorReading() < minThresh )
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
