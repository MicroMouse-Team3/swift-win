
#include <Motor.h>
#include <EncoderMM.h>
#include <Sensor.h>
#include <LED.h>
#include <Functions.h>

/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015. Hi
**/

byte LpwmA = 50;
byte LpwmB = 0;
byte RpwmA = 50;
byte RpwmB = 0;

#include <math.h>

void setup(){
  Serial.begin(9600); //Used for Debugging
  if(debugOn){
    Serial.println("->setup");
  }

  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
  
  //Left left
  sensor[0] = new Sensor( leftEmitIR , leftRecIR , leftLED );
  //Left Diag
  sensor[1] = new Sensor( leftDiagEmitIR , leftDiagRecIR , leftDiagLED );
  //Left Front
  sensor[2] = new Sensor( leftFrontEmitIR , leftDiagRecIR , leftDiagLED );
  //Right Right
  sensor[3] = new Sensor( rightEmitIR , rightRecIR , rightLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right Front
  sensor[5] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED ); 
  
  attachInterrupt( L_CH_A , incEncoderL , CHANGE );
  attachInterrupt( R_CH_A , incEncoderR , CHANGE );
  
  delay(7000);
  //setBothMtrsForward();
  mtrL->setForward( LpwmA , LpwmB );
  mtrR->setForward( RpwmA , RpwmB );
}

void loop(){
  if(debugOn){
    Serial.println("->Loop"); //Used for Debugging 
  }
  
  hopeEyeNeverHitWall();
  
  PID();
}

void   hopeEyeNeverHitWall() {
  if ( sensor[2]->getIR() < minThresh || sensor[3]->getIR() < minThresh ) {    
    if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() < minThresh )
      turnAround();
    else if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() > minThresh )
      turnRight();
    else if ( sensor[0]->getIR() > minThresh && sensor[5]->getIR() < minThresh )
      turnLeft();
    else
      fullStop();
  }
  else
    PID();      //Keep going straight
}

void turnAround() {
  //turnRight
  turnRight();
  turnRight();
  fullStop();
  //overshoot?
}
void turnRight() {
  LpwmA = RpwmA = 10;
  LpwmB = RpwmB = 10;
  mtrL->setForward( LpwmA , LpwmB );
  mtrR->setBackward( RpwmA , RpwmB );
  delay(10);
  //overshoot?
  fullStop();
}
void turnLeft() {
  mtrR->setForward( RpwmA , RpwmB );
  mtrL->setBackward( LpwmA , LpwmB );
  delay(10);
  //overshoot?
  fullStop();
}

void fullStop() {
  mtrR->setForward( 0 , 0 );
  mtrL->setForward( 0 , 0 );  
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
  
  int left = sensor[0]->getIR();//getLeftIR();
  int right = sensor[5]->getIR();//getRightIR();
  int diff = 0;
  
  diff = left - right;
  
  if (diff >= 20){
    byte speed = 255;
    setBothMtrsForward();
    setBothMtrsSpd(speed = 255);
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

void goStraight(int leftPWM, int rightPWM){
  setBothMtrsForward();
   if(debugOn){
     Serial.println("->goStraight"); //Used for Debugging 
  }
  mtrL->setForward( leftPWM , LOW );
  mtrR->setForward( rightPWM , LOW );
  
  /*
  mtrL->setSpeed(leftPWM);
  mtrR->setSpeed(rightPWM);
  */
//  analogWrite(RpwmA, rightpwm);
//  analogWrite(LpwmA, leftpwm);
}


int distanceTraveled(){
  if(debugOn){
    Serial.println("->distanceTraveled"); //Used for Debugging 
  }
  return 0;
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

  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->getIR();
}
