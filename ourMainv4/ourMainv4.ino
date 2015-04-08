
#include <Motor.h>
#include <Sensor.h>
#include <LED.h>
#include <EncoderMM.h>
#include <Functions.h>
#include <math.h>

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
  
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
  delay(4000);

  mtrL->setForward( 150 , 0 );
  mtrR->setForward( 150 , 0 );
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

  if ( mapMode ) {
//    mapMaze();
  }
  
  //hopeEyeNeverHitWall();
  
  for ( int i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getLED().setHIGH();
    delay(100);
  }
  delay(500);
    for ( int i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getLED().setLOW();
    delay(100);  
  }
  
  for ( int i = 0 ; i < 4 ; i++ ) {
    squareTest();
  }
  delay(2000);
//  PID();
}

int blockLength = 10;

void mapMaze() {
   maze[0][0] = -1;
   encTickL = encTickR = 0;
             
}

void squareTest() {
  fullStraight();
  delay(100);
  turnRight();
  delay(100);
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
  else
    PID();      //Keep going straight
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
  fullStop();
  delay(1000); 
  LpwmA = RpwmA = 100;
  LpwmB = RpwmB = 0;
  mtrL->setForward( 150 , 0 );
  mtrR->setBackward( 0 , 150 );
  delay(50);
//  encTickL = encTickR = 0;
//  while ( encTickL < 256 );
  fullStop();
  delay(1000);
  //overshoot?
 // fullStop();
  //delay(1000);
  //fullStraight();
}
void turnLeft() {
  fullStop();
  delay(1000);
  LpwmA = RpwmA = 100;
  LpwmB = RpwmB = 0;
  mtrR->setForward( RpwmA , RpwmB );
  mtrL->setBackward( LpwmA , LpwmB );
  delay(10);
//  encTickL = encTickR = 0;
//  while ( encTickR < 256 );  
  //overshoot?
  fullStop();
  delay(1000);
}
void fullStop() {
  mtrR->setForward( 0 , 0 );
  mtrL->setForward( 0 , 0 );  
}
void fullStraight() {
  RpwmA = LpwmA = 150;
  RpwmB = LpwmB = 0;
  mtrR->setForward( 150 , 0 );
  mtrL->setForward( 150 , 0 );
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
  
  mtrL->setForward( outputSpeed , 0 );
  mtrR->setForward( outputSpeed , 0 );
}


void readDasSensors(){
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }

  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->getIR();
}
