#include <Motor.h>
#include <Sensor.h>
#include <EncoderMM.h>
#include <LED.h>

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

const int NUMSENSORS = 6;
volatile static int encTickL = 0, encTickR = 0;

Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];


//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
long int delayLastError = 0;
int previous_error = 0;
unsigned long curt = 0; 

//Encoder Information
volatile int state = LOW;




 
 
//SENSOR Threshold Values
unsigned int minThresh = 15;
unsigned int maxThresh = 700;



/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015. Hi
**/

byte LpwmA = 50;
byte LpwmB = 0;
byte RpwmA = 50;
byte RpwmB = 0;

int lastTime = 0;
int lastError = 0;

void setup(){
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, INPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, INPUT);
  pinMode(25, OUTPUT);
  pinMode(26, INPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, INPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  
  
  
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
  
  
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
  
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
  delay(5000);

  digitalWrite(leftLED, HIGH);
  digitalWrite(leftDiagLED, HIGH);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  digitalWrite(rightDiagLED, HIGH);
  digitalWrite(rightLED, HIGH);
  
  delay(5000);
  
  digitalWrite(leftLED, LOW);
  digitalWrite(leftDiagLED, LOW);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  digitalWrite(rightDiagLED, LOW);
  digitalWrite(rightLED, LOW);
  
  //Traps Setup until both front sensors are higher than 600.
  digitalWrite(rightFrontLED, HIGH);
  digitalWrite(leftFrontLED, LOW);
  delay(500);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, HIGH);
  delay(500);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, LOW);
  delay(500);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, HIGH);
  delay(500);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  
  while ((sensor[2]->getIR() < 800) && (sensor[3]->getIR() < 800)){
   digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  delay(128);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  delay(128);
  }
  
  encTickL = 0;
  encTickR = 0;
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  delay(500);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  delay(500);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  delay(500);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
}

boolean mapMode = true;
int maze[16][16] = { { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }, 
                                    { 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 4 , 4 , 4 , 4 , 4 , 4 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 5 , 5 , 5 , 5 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 6 , 6 , 6 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 7 , 6 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 7 , 6 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 6 , 6 , 6 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 5 , 5 , 5 , 5 , 5 , 5 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 4 , 4 , 4 , 4 , 4 , 4 , 4 , 4 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 2 , 1 , 0 }, 
                                    { 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 }, 
                                    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } };
                                    
                                    
void loop(){  
 /* while (encTickR < 1100){
    mtrL->setForward( 128 );
    mtrR->setForward( 128 );
  }
  while (encTickR < 1334){
    mtrL->setBackward( 255 );
    mtrR->setBackward( 255 );
  }*/
  encTickR = 0;
  encTickL = 0;
  mtrL->setForward( 0 );
  mtrR->setForward( 0 );
  delay(5000);
  
  mtrL->setForward(128);
  mtrR->setForward(128);
  
  while(encTickR < 1334){
  if(current_error() > 0){
    mtrL->setBackWards( P_error() + D_Error());
    mtrR->setBackWards( P_error() + D_Error());
    }
  }
  
  
}
int current_error(){
  int curPos = (encTickR + encTickL)/2
  int error = curPos - 1334;
  previous_error = error;
  return error;
}
int P_error(){
  delayLastError = millis();
  return current_error() * 1;
}
int D_error(){
  return (current_error() - previous_error)/(millis() - delayLastError); 
}
void testSensors(){
  int test1 = sensor[0]->getIR();
  int test2 = sensor[1]->getIR();
  int test3 = sensor[2]->getIR();
  int test4 = sensor[3]->getIR();
  int test5 = sensor[4]->getIR();
  int test6 = sensor[5]->getIR();
  delay(5000);
}
void LEDsON() {
  digitalWrite(rightLED, HIGH); delay(50);
        digitalWrite(rightFrontLED,HIGH); delay(50);
        digitalWrite(rightDiagLED,HIGH); delay(50);
        digitalWrite(leftLED, HIGH); delay(50);
        digitalWrite(leftFrontLED, HIGH); delay(50);
        digitalWrite(leftDiagLED, HIGH); delay(50);
}
void LEDsOFF() {
         digitalWrite(rightLED, LOW); delay(50);
        digitalWrite(rightFrontLED,LOW); delay(50);
        digitalWrite(rightDiagLED,LOW); delay(50);
        digitalWrite(leftLED, LOW); delay(50);
        digitalWrite(leftFrontLED, LOW); delay(50);
        digitalWrite(leftDiagLED, LOW); delay(50);
}

int blockLength = 10;

void mapMaze() {
   maze[0][0] = -1;
   encTickL = encTickR = 0;             
}

void squareTest() {
fullStraight();
  delay(200);
  fullStop();
  delay(50);
  turnRight();
  delay(50);
  fullStop();
  delay(500);  
}

void fullStop() {

}
void fullStraight() {
  
}

void turnAround() {
  //turnRight
  //delay(10);
  turnRight();
  turnRight();
//  fullStop();
  //overshoot?
}
void turnRight() { 
  
}
void turnLeft() {
  
}



void   hopeEyeNeverHitWall() {
  //Left = 0, LeftDiag = 1, LeftFront = 2, RightFront = 3, RightDiag = 4, Right = 5
  if ( sensor[2]->getIR() < minThresh || sensor[3]->getIR() < minThresh ) {     
     //Turn around if walls on left and right sides.    
    if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() < minThresh ) {
      turnAround();
      fullStraight();
    }
      
     //Turn Right if wall on left and NOT on right 
    else if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() > minThresh ) {
      turnRight();
      fullStraight();
    }
      
      //Turn left if wall on right and NOT on left
    else if ( sensor[0]->getIR() > minThresh && sensor[5]->getIR() < minThresh ) {
      turnLeft();
      fullStraight();
    }
    else
      fullStop();
    //blink all LEDs and freak the hell out
  }
 // else
  //  PID();      //Keep going straight
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
  
  int error;
  int outputSpeed = 0;
  int integral =0, proportion=0, derivative=0;
  
  int KP = 1;
  int KI = 1;
  int KD = 1;
  
  int left = sensor[0]->getIR();//getLeftIR();
  int right = sensor[5]->getIR();//getRightIR();
  int diff = 0;
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;
  
  diff = left - right;
  
 
  error = diff;
  
  //Calculating the output
  proportion = error * KP;
  integral += error*deltaTime;
  integral *= KI;   
  derivative = ((error-lastError)/deltaTime)*KD;   
  outputSpeed = proportion + integral + derivative;
  
  lastError = error;
  lastTime = currentTime;
  
  mtrL->setForward( outputSpeed);
  mtrR->setForward( outputSpeed);
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

