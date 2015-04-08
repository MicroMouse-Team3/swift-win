
#include <Motor.h>
#include <Sensor.h>
#include <LED.h>
#include <EncoderMM.h>
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
  Serial.println("->setup");
  
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
  
  //Left left
  sensor[0] = new Sensor( leftEmitIR , leftRecIR , leftLED );
  //Left Diag
  sensor[1] = new Sensor( leftDiagEmitIR , leftDiagRecIR , leftDiagLED );
  //Left Front
  sensor[2] = new Sensor( leftFrontEmitIR , leftDiagRecIR , leftDiagLED );
  //Right Front
  sensor[3] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right Right
  sensor[5] = new Sensor( rightEmitIR , rightRecIR , rightLED ); 
  
  attachInterrupt( L_CH_A , incEncoderL , CHANGE );
  attachInterrupt( R_CH_A , incEncoderR , CHANGE );
  
  delay(7000);

  mtrL->setForward( LpwmA , LpwmB );
  mtrR->setForward( RpwmA , RpwmB );
}

void loop(){
  
  hopeEyeNeverHitWall();
  
//  PID();
}

void   hopeEyeNeverHitWall() {
  //Left = 0, LeftDiag = 1, LeftFront = 2, RightFront = 3, RightDiag = 4, Right = 5
  if ( sensor[2]->getIR() < minThresh || sensor[3]->getIR() < minThresh ) {     
     //Turn around if walls on left and right sides.    
    if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() < minThresh )
      turnAround();
      
     //Turn Right if wall on left and NOT on right 
    else if ( sensor[0]->getIR() < minThresh && sensor[5]->getIR() > minThresh )
      turnRight();
      
      //Turn left if wall on right and NOT on left
    else if ( sensor[0]->getIR() > minThresh && sensor[5]->getIR() < minThresh )
      turnLeft();
    else
      fullStop();
    //blink all LEDs and freak the hell out
  }
  else
    PID();      //Keep going straight
}

void turnAround() {
  //turnRight
  turnRight();
  delay(10);
  turnRight();
  fullStop();
  //overshoot?
  delay(1000);
  fullStraight();
}
void turnRight() {
  LpwmA = RpwmA = 10;
  LpwmB = RpwmB = 0;
  mtrL->setForward( LpwmA , LpwmB );
  mtrR->setBackward( RpwmA , RpwmB );
  delay(10);
  //overshoot?
  fullStop();
  delay(1000);
  fullStraight();
}
void turnLeft() {
  LpwmA = RpwmA = 10;
  LpwmB = RpwmB = 0;
  mtrR->setForward( RpwmA , RpwmB );
  mtrL->setBackward( LpwmA , LpwmB );
  delay(10);
  //overshoot?
  fullStop();
  delay(1000);
  fullStraight();
}
void fullStop() {
  mtrR->setForward( 0 , 0 );
  mtrL->setForward( 0 , 0 );  
}
void fullStraight() {
  RpwmA = LpwmA = 10;
  RpwmB = LpwmB = 0;
  mtrR->setForward( RpwmA , RpwmB );
  mtrL->setForward( LpwmA , LpwmB );
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
//    goStraight(distancePerMove, distancePerMove+1);
  }  
  else if (diff < -20){
  //  goStraight(distancePerMove+1, distancePerMove);
  }
  else{
    //goStraight(distancePerMove, distancePerMove);
  }
}


void readDasSensors(){
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }

  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->getIR();
}
