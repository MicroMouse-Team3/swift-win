
#ifndef MMvars_h
#define MMvars_h

#include "Arduino.h"

/*
extern boolean frontWall;
extern boolean leftWall;
extern boolean leftFrontWall;
extern boolean rightWall;
extern boolean rightFrontWall;
extern boolean myBool;
extern boolean yourBool;
*/

//Left Sensors and LEDS
#define leftEmitIR 3      
#define leftRecIR A12
#define leftLED 11
#define leftDiagEmitIR 28
#define leftDiagRecIR A18
#define leftDiagLED 30
#define leftFrontEmitIR 4  
#define leftFrontRecIR A11
#define leftFrontLED 13  

//Right Sensors and= LEDs
#define rightEmitIR 2 
#define rightRecIR A13
#define rightLED 12
#define rightDiagEmitIR 25
#define rightDiagRecIR A15
#define rightDiagLED 27
#define rightFrontEmitIR 5
#define rightFrontRecIR A10
#define rightFrontLED 14

//Enable Pins
#define L_Enable 16
#define R_Enable 17

//Motor H-Bridge Pins
#define L_Mtr_A 20
#define L_Mtr_B 21
#define R_Mtr_A 22
#define R_Mtr_B 23

//Encoder Pins
#define L_CH_A 9
#define L_CH_B 10
#define R_CH_A 7
#define R_CH_B 8


unsigned long lastTickLeft = 0;
unsigned long lastTickRight = 0;

const byte NUMSENSORS = 6;
volatile static int encTickL = 0, encTickR = 0;
int ourOffset = 0;

Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];


//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
float previousError = 0.0;
int error = 0;
int curTime = 0;
int lastSamp = 0;
int delayTime = 0;
float previousPos = 0.0;
int previousTime = 0;
unsigned long curt = 0; 
float stopError = 0.0;
int kp = 1;

int lastTime = millis();
int lastError = 0;

//Encoder Information
volatile int state = LOW;

//Speeds of motors
int mapSpeed = 100;
int solveSpeed = 255;
 
 
//SENSOR Threshold Values
unsigned int minThresh = 15;
unsigned int maxThresh = 700;

bool wallLeft;
bool wallFront;
bool wallRight;
  
#endif
