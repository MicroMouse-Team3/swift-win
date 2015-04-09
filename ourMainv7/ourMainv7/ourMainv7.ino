#include <Motor.h>
#include <Sensor.h>
#include <EncoderMM.h>
//#include <Functions.h>
#include <LED.h>

const int NUMSENSORS = 6;

#define leftRecIR A12
#define leftDiagRecIR A18

//Pin Assignments for IR Sensors
#define leftEmitIR 3
#define leftLED 11
#define leftDiagEmitIR 28
#define leftDiagLED 30
#define leftFrontEmitIR 4
#define leftFrontRecIR A11
#define leftFrontLED 13

#define rightEmitIR 2 
#define rightRecIR A13
#define rightLED 12
#define rightDiagEmitIR 25
#define rightDiagRecIR A15
#define rightDiagLED 27
#define rightFrontEmitIR 5
#define rightFrontRecIR A10
#define rightFrontLED 14

/********** LEFT MOTOR VARS **********/

//Left Enable            // Connection correlation to MM schematic
const byte L_Enable = 16;  // (H-Bridge pin 1)

//Motors
const byte L_Mtr_A = 20;  // (H-Bridge pin 2)  //1A
const byte L_Mtr_B = 21;  // (H-Bridge pin 7)  //2A

//Encoder vars
const byte L_CH_A = 9;          // (H-Bridge SV3 pin 6)
const byte L_CH_B = 10;         // (H-Bridge SV3 pin 5)

/********** RIGHT MOTOR VARS **********/

//Right Enable            // Connection correlation to MM schematic
const int R_Enable = 17;  // (H-Bridge pin ?)

//Motors
const int R_Mtr_A = 22;  // (H-Bridge pin ?)  //3A
const int R_Mtr_B = 23;  // (H-Bridge pin ?)  //4A

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
  Serial.begin(9600); //Used for Debugging
  Serial.println("->setup");
 


  
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
  sensor[2] = new Sensor( leftFrontEmitIR , leftDiagRecIR , leftDiagLED );
  //Right Front
  sensor[3] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right Right
  sensor[5] = new Sensor( rightEmitIR , rightRecIR , rightLED ); 

  
  pinMode( leftLED , OUTPUT );
  pinMode( leftDiagLED , OUTPUT );
  pinMode( leftFrontLED , OUTPUT );
  pinMode( rightFrontLED , OUTPUT );
  pinMode( rightDiagLED , OUTPUT );
  pinMode( rightFrontLED , OUTPUT );
  
          
            
  
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
  delay(5000);

  setBothMtrsForward( 100 , 0 );
  setBothMtrsSpd(100);
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
  delay(3000);
  
  hopeEyeNeverHitWall();

  for ( int i = 0 ; i < NUMSENSORS ; i++ ) {        
 //   LEDsON();     
    sensor[i]->getLED().setHIGH();
    delay(500);
  }
  //delay(500);
  for ( int i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getLED().setLOW();
    
  // LEDsOFF();     
    delay(500);  
  }

  for ( int i = 0 ; i < 4 ; i++ ) {
    squareTest();

  }
  
//  PID();
}

void LEDsON() {
  digitalWrite(rightLED, HIGH); delay(500);
        digitalWrite(rightFrontLED,HIGH); delay(500);
        digitalWrite(rightDiagLED,HIGH); delay(500);
        digitalWrite(leftLED, HIGH); delay(500);
        digitalWrite(leftFrontLED, HIGH);        delay(500);
        digitalWrite(leftDiagLED, HIGH);        delay(500);
}
void LEDsOFF() {
         digitalWrite(rightLED, LOW);        delay(500);
        digitalWrite(rightFrontLED,LOW);        delay(500);
        digitalWrite(rightDiagLED,LOW);        delay(500);
        digitalWrite(leftLED, LOW);        delay(500);
        digitalWrite(leftFrontLED, LOW);        delay(500);
        digitalWrite(leftDiagLED, LOW);        delay(500);
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

  setBothMtrsForward( 0 , 0 );
  setBothMtrsSpd(0);
}
void fullStraight() {
  RpwmA = LpwmA = 150;
  RpwmB = LpwmB = 0;  
  setBothMtrsForward( 150 , 0 );
  setBothMtrsSpd(200);
  
  delay(50);
  fullStop();
  delay(1000);
//  mtrR->setForward( 150 , 0 );
//  mtrL->setForward( 150 , 0 );
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
  setMtrsRightTurn( 0 , 150 );
  setBothMtrsSpd(200);
//  mtrL->setForward( 150 , 0 );
//  mtrR->setBackward( 0 , 150 );
  delay(70);
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
  setMtrsLeftTurn( 150 , 0 );
  setBothMtrsSpd(200);
//  mtrR->setForward( RpwmA , RpwmB );
//  mtrL->setBackward( LpwmA , LpwmB );
  delay(500);
//  encTickL = encTickR = 0;
//  while ( encTickR < 256 );  
  //overshoot?
  fullStop();
  delay(1000);
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
/*
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
*/
/*

void readDasSensors(){
  if(debugOn){
    Serial.println("->readSensors"); //Used for Debugging 
  }

  for ( byte i = 0 ; i < NUMSENSORS ; i++ )
    sensor[i]->getIR();
}

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
*/
/********** MOTOR FUNCTIONS **********/

void setLMtrSpd( byte speed ) {
  mtrL->setSpeed(speed);     
}

void setRMtrSpd( byte speed ) {
  mtrR->setSpeed(speed);
}

void setBothMtrsSpd( byte speed ) {
  setLMtrSpd(speed);
  setRMtrSpd(speed);
  //mtrL->setSpeed(speed);
  //mtrR->setSpeed(speed);
}

void setBothMtrsForward( byte spdA , byte spdB ) {
  mtrL->setForward( 150 , 0 );
  mtrR->setForward( 150 , 0 );
}

void setBothMtrsBackward( byte spdA , byte spdB ) {
  mtrL->setBackward( 0 , 150 );
  mtrR->setBackward( 0 , 150 );
}

void setMtrsLeftTurn( byte spdA , byte spdB ) {
  mtrL->setForward( 0 , 150 );
  mtrR->setBackward( 0 , 150 );
}

void setMtrsRightTurn( byte spdA , byte spdB ) {
  mtrL->setForward( 150 , 0 );
  mtrR->setBackward( 150 , 0 );
  
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

