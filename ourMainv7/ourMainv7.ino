#include <Motor.h>
#include <Sensor.h>
#include <EncoderMM.h>
//#include <Functions.h>
#include <LED.h>

#define NUMSENSORS 6
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
#define L_Enable 16  // (H-Bridge pin 1)

//Motors
#define L_Mtr_A 20  // (H-Bridge pin 2)  //1A
#define L_Mtr_B 21  // (H-Bridge pin 7)  //2A

//Encoder vars
#define L_CH_A 9         // (H-Bridge SV3 pin 6)
#define L_CH_B 10         // (H-Bridge SV3 pin 5)

/********** RIGHT MOTOR VARS **********/

//Right Enable            // Connection correlation to MM schematic
#define R_Enable 17  // (H-Bridge pin ?)

//Motors
#define R_Mtr_A 22  // (H-Bridge pin ?)  //3A
#define R_Mtr_B 23  // (H-Bridge pin ?)  //4A

//Encoder vars
#define R_CH_A 7          // (H-Bridge SV3 pin ?)
#define R_CH_B 8         // (H-Bridge SV3 pin ?)


int encTickL = 0, encTickR = 0;

//unsigned long curt = 0; 



Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];

/*
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
 */
 
//SENSOR Threshold Values
unsigned int minThresh = 650;
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
  
  
  pinMode( L_Enable , OUTPUT ); pinMode( L_Mtr_A , OUTPUT ); pinMode( L_Mtr_B , OUTPUT );
  pinMode( R_Enable , OUTPUT ); pinMode( R_Mtr_A , OUTPUT ); pinMode( R_Mtr_B , OUTPUT );
//  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
//  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );

  pinMode( leftLED , OUTPUT ); pinMode( leftDiagLED , OUTPUT ); pinMode( leftFrontLED , OUTPUT );
  pinMode( rightFrontLED , OUTPUT ); pinMode( rightDiagLED , OUTPUT ); pinMode( rightFrontLED , OUTPUT );

  pinMode( leftEmitIR , OUTPUT ); pinMode( leftDiagEmitIR , OUTPUT ); pinMode( leftFrontEmitIR , OUTPUT );
  pinMode( rightEmitIR , OUTPUT ); pinMode( rightEmitIR , OUTPUT ); pinMode( rightFrontEmitIR , OUTPUT );  
  
  pinMode( leftRecIR , INPUT ); pinMode( leftDiagRecIR , INPUT ); pinMode( leftFrontRecIR , INPUT );
  pinMode( rightRecIR , INPUT ); pinMode( rightRecIR , INPUT ); pinMode( rightFrontRecIR , INPUT );  
  
                       
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
  delay(4000);

  //mtrL->setForward( 150 , 0 );
  //mtrR->setForward( 150 , 0 );
  setBothMtrsForward( 200 , 0 );
  setBothMtrsSpd(200);
//  mtrL->setSpeed( 200 );
//  mtrR->setSpeed( 200 );
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

  
  //hopeEyeNeverHitWall();
  
  //for ( int i = 0 ; i < NUMSENSORS ; i++ ) {
                
        digitalWrite(rightLED, HIGH); delay(500); digitalWrite(rightLED, LOW); delay(500); 
        digitalWrite(rightFrontLED,HIGH); delay(500); digitalWrite(rightFrontLED, LOW); delay(500);
        digitalWrite(rightDiagLED,HIGH); delay(500);
        digitalWrite(leftLED, HIGH); delay(500); digitalWrite(leftLED, LOW); delay(500);
        digitalWrite(leftFrontLED, HIGH); delay(500);
        digitalWrite(leftDiagLED, HIGH); delay(500);

  //for ( int i = 0 ; i < NUMSENSORS ; i++ ) {
    //sensor[i]->getLED().setLOW();

        
        digitalWrite(rightDiagLED,LOW);        delay(500);
        
        digitalWrite(leftFrontLED, LOW);        delay(500);
        digitalWrite(leftDiagLED, LOW);        delay(500);

  //}
  /*
  for ( int i = 0 ; i < 4 ; i++ ) {
    squareTest();
  }
  delay(2000);
  */
//  PID();
}

int blockLength = 10;

void mapMaze() {
   maze[0][0] = -1;
   encTickL = encTickR = 0;
             
}

void squareTest() {
  fullStraight();
  turnRight();
//  delay(100);
}

void fullStop() {

  setBothMtrsForward( 0 , 0 );
  setBothMtrsSpd(0);
//  mtrR->setForward( 0 , 0 );
//  mtrL->setForward( 0 , 0 );  
}
void fullStraight() {
  RpwmA = LpwmA = 150;
  RpwmB = LpwmB = 0;  
  setBothMtrsForward( 150 , 0 );
  setBothMtrsSpd(200);
  
  delay(300);
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
  setMtrsRightTurn( 150 , 0 );
  setBothMtrsSpd(200);
//  mtrL->setForward( 150 , 0 );
//  mtrR->setBackward( 0 , 150 );
  delay(1000);
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
  setMtrsLeftTurn( 0 , 150 );
  setBothMtrsSpd(200);
//  mtrR->setForward( RpwmA , RpwmB );
//  mtrL->setBackward( LpwmA , LpwmB );
  delay(1000);
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
 //   PID();      //Keep going straight
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
*/
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
   analogWrite( L_Enable , speed );
 // mtrL->setSpeed(speed);     
}

void setRMtrSpd( byte speed ) {
  analogWrite( R_Enable , speed );
//  mtrR->setSpeed(speed);
}

void setBothMtrsSpd( byte speed ) {
 
  
  
  setLMtrSpd(speed);
  setRMtrSpd(speed);
 
  //mtrL->setSpeed(speed);
  //mtrR->setSpeed(speed);
}

void setBothMtrsForward( byte spdA , byte spdB ) {
  analogWrite( L_Mtr_A , spdA );
  analogWrite( L_Mtr_B , spdB );
  analogWrite( R_Mtr_A , spdA);
  analogWrite( R_Mtr_B , spdB );

  /*
  mtrL->setForward( spdA , spdB );
  mtrR->setForward( spdA , spdB );
  */
}

void setBothMtrsBackward( byte spdA , byte spdB ) {
  analogWrite( L_Mtr_A , spdB );
  analogWrite( L_Mtr_B , spdA );
  analogWrite( R_Mtr_A , spdB);
  analogWrite( R_Mtr_B , spdA );
/*
  mtrL->setBackward( spdA , spdB );
  mtrR->setBackward( spdA , spdB );
  */
}

void setMtrsLeftTurn( byte spdA , byte spdB ) {
  analogWrite( L_Mtr_A , spdA );
  analogWrite( L_Mtr_B , spdB );
  analogWrite( R_Mtr_A , spdB);
  analogWrite( R_Mtr_B , spdA );
  /*
  mtrL->setBackward( spdB , spdA );
  mtrR->setForward( spdA , spdB );
  */
}

void setMtrsRightTurn( byte spdA , byte spdB ) {
  analogWrite( L_Mtr_A , spdB );
  analogWrite( L_Mtr_B , spdA );
  analogWrite( R_Mtr_A , spdA);
  analogWrite( R_Mtr_B , spdB );

  mtrL->setForward( spdA , spdB );
  mtrR->setBackward( spdB , spdA );
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

