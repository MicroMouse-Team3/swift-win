#include <Encoder.h>

//TABLE OF CONTENTS (SEARCH FOR THESE TERMS)
//////////////////////////////
//Defintions -> DEFINEME
//Initilizations -> INITME
//Setup -> SETME
//Loop -> LOOPME
//Turn -> TURNME
//Speed Control -> SPEEDME
//PID -> PIDME
//EncoderTick -> TICKME
//Error Functions -> ERRME
//Get Sensor Functions -> SENME
//--------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Definitions
//Search Term: DEFINEME
//Left Sensors and LEDS

int floodFillNum = 0, currentFFVal;
int xPrev, yPrev, posX, posY;
#define MAXSIZE 255

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

//Direction
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define LEFT 0
#define STRAIGHT 1
#define RIGHT 2
#define UTURN 3

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int mazeMap[16][16] = {{-1},{-1}};       
byte FFval = 255;

int navDir;
int dx = 0, dy = 1;
int wallX = 1, wallY = 31;
//Initializations
//INITME
//For Mapping
int orientation = 4000;
int x = 0;
int y = 0;
int xprev = 0;
int yprev = 0;
const bool solved = "FALSE";

bool mapMode = true;
bool wallLeft = false;
bool wallLeftDiag = false;
bool wallFront = false;
bool wallLeftFront = false;
bool wallRightFront = false;
bool wallRightDiag = false;
bool wallRight = false; 

//For Movement
const int distancePerMove = 30;
int mapSpeed = 128;
int solveSpeed = 255; 
int ticksForTurn = 748;
double curAccel;
double curVelX;


//For PID

// PID gains
int Tkp = 1;  // 1.86 * 1000
int Tkd = 100;  // 0.086 * 1000 * 1000000***********************************************************
double oldPError;

// Error Tracking
double errOld = 0;
double error = 0;
double currTime = 0;
double lastTime =0;
double delayTime = 0;
int nextTurn = 0;
double deltaTime = 0;
long Time = 0;
long prevTime = 0;
double oldCurVelX;

// Setpoints
int cellDistance = 2200;
int setPoint = cellDistance;
int setVal = 0;
int wallOffSet = 123;

// Sensor offsets
double wallLeftDist = 0;
double wallRightDist = 0;

//PWM vars

int pwmRate = 0;


//For Sensors
const int NUMSENSORS = 6;
unsigned int minThresh = 15;
unsigned int maxThresh = 700;
int ourOffset = 0;

//For Encoders
volatile static int encTickL = 0, encTickR = 0;
volatile int state = LOW;
unsigned long lastTickLeft = 0;
unsigned long lastTickRight = 0;
double leftEncoderChange;
double leftEncoderOld;
double rightEncoderChange;
double rightEncoderOld;
double encoderChange;

//Maze
byte maze[16][16] =  { { 14 , 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14 }, 
                       { 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  6 ,  1 ,  8 ,  9 , 10 , 11 , 12 , 13 }, 
                       { 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  5 ,  2 ,  7 ,  8 ,  9 , 10 , 11 , 12 }, 
                       { 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  4 ,  3 ,  6 ,  7 ,  8 ,  9 , 10 , 11 }, 
                       { 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 }, 
                       {  9 ,  8 ,  7 ,  6 ,  5 ,  5 ,  5 ,  2 ,  2 ,  5 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 }, 
                       {  8 ,  7 ,  6 ,  5 ,  4 ,  5 ,  2 ,  1 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 }, 
                       {  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  0 ,  0 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 }, 
                       {  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  0 ,  0 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 }, 
                       {  8 ,  7 ,  2 ,  3 ,  4 ,  5 ,  2 ,  1 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 }, 
                       {  9 ,  8 ,  7 ,  3 ,  4 ,  3 ,  5 ,  2 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 }, 
                       { 10 ,  9 ,  8 ,  7 ,  4 ,  4 ,  4 ,  3 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 }, 
                       { 11 , 10 ,  9 ,  8 ,  7 ,  3 ,  3 ,  4 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 }, 
                       { 12 , 11 , 10 ,  9 ,  8 ,  7 ,  2 ,  5 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 }, 
                       { 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 }, 
                       { 14 , 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14 } };                  
                       
int floodFillMap[16][16] =  { {-1,}, {-1} };                       
                       
byte wallMap[33][33] =  { 
                          {'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1' }, 
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},                        
	                  {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'}, 
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'}, 
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'}, 
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'}, 
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'}, 
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'}, 
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'}, 
                          {'1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'}, 
                          {'1','0','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'},
                          {'1','0','1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},
                          {'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'} };
                                      

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Function
//SETME
void setup(){

  //initializePins
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
  
  // TROUBLESHOOTING
  Serial.begin(9600);
  
  
   //attach interrupts
  //attachInterrupt( L_CH_A , incEncoderL , RISING );
  //attachInterrupt( R_CH_A , incEncoderR , RISING );
  Encoder leftEnc(L_CH_A, L_CH_B);
  Encoder rightEnc(R_CH_A, R_CH_B);
  

  //Set Enables High
  digitalWrite(L_Enable, HIGH);
  digitalWrite(R_Enable, HIGH);
  
  //LED Test
  delay(5000);
  digitalWrite(leftLED, HIGH);
  digitalWrite(leftDiagLED, HIGH);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  digitalWrite(rightDiagLED, HIGH);
  digitalWrite(rightLED, HIGH);
  delay(2000);
  digitalWrite(leftLED, LOW);
  digitalWrite(leftDiagLED, LOW);
  digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  digitalWrite(rightDiagLED, LOW);
  digitalWrite(rightLED, LOW);
  
  //Wait for Startup
  while ((getIRLeftFront() < 800) && (getIRRightFront() < 800)){
   digitalWrite(leftFrontLED, LOW);
  digitalWrite(rightFrontLED, LOW);
  delay(128);
  digitalWrite(leftFrontLED, HIGH);
  digitalWrite(rightFrontLED, HIGH);
  delay(128);
  }
  
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
  
  //Determine Inital Offset

  lastTime = micros();
  wallLeftDist = getIRLeft();
  wallRightDist = getIRRight();
  ourOffset = wallRightDist - wallLeftDist;
  
  leftEnc.write(0);
  rightEnc.write(0);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Loop Functions
//Search Term: LOOPME
void loop(){
  
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Speed Control Function
//Search Term: SPEEDME
int speedControl(){
  errOld = error;
  error = (cellDistance) - (encTickR+encTickL)/2;
  pwmRate = Tkp * error + Tkd * (error - errOld);
  return pwmRate;
}

// Gets encoder information for speed controller
void getEncoderStatus(){
  leftEncoderChange = encTickL - leftEncoderOld;
  rightEncoderChange = encTickR - rightEncoderOld;
  encoderChange = (leftEncoderChange + rightEncoderChange)/2;
  leftEncoderOld = encTickL;
  rightEncoderOld = encTickR; 
}

// New speed controller function
int calculateSpeed(int desiredVel){
  double pError = 0;
  double posPWMX = 0; 
  curVelX =  encoderChange * 1000;
  pError = desiredVel - curVelX;
  posPWMX = pError + (pError - oldPError);
  oldPError = pError;
  return posPWMX;
}
  
// Will be used in speed profile function
int calcAccel(){
   currTime = micros();
   deltaTime = currTime - lastTime;
   lastTime = currTime;
   curAccel = ((curVelX - oldCurVelX)/deltaTime);
   oldCurVelX = curVelX;
   return curAccel;
} 

int wallControl(){
  //Needs to be able to stop when very close to wall.
  
  int distanceToStop = 1000; //random value to be used. This will become how close we want to stop in front of the wall.
  int currentDistance = getIRRightFront();
  
  errOld = error;
  error = distanceToStop - currentDistance;
  pwmRate = Tkp * error;
  pwmRate += Tkd * (error - errOld);
  return pwmRate;
}

int turnControl(){
  //Needs to be able to stop when very close to 90 degrees.
  
  errOld = error;
  error = ticksForTurn - ((encTickR + encTickL)/2);
  pwmRate = Tkp * error;
  currTime = micros();
  deltaTime = currTime - lastTime;
  pwmRate += Tkd * (error - errOld)/deltaTime;
  lastTime = currTime;
  return pwmRate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID Function
//Search Term: PIDME
void PID(){
  int error = 0;
  int errorP = 0;
  int errorD = 0;
  static int oldErrorP = 0;
  int pwmPlus = 0;
  int pwmMinus = 0;
  unsigned int KP = 1;
  unsigned int KD = 1;
  
  int left = getIRLeft();
  int right = getIRRight();
  
  if (wallLeft && wallRight){
     errorP = right - left - ourOffset;
     errorD = errorP - oldErrorP;
  }
  
  else if(left){
     errorP = 0; //(wallLeftDist - left);
     errorD = 0;
  }
  
  else if(right){
     errorP = 0; //(right - wallRightDist);
     errorD = 0;
  }
  
  else if (!left && !right){
     errorP = 0;
     errorD = 0; 
  }
  
  error = (KP * errorP) + (KD * errorD);
  oldErrorP = errorP;
  
  pwmPlus = pwmRate + error;
  pwmMinus = pwmRate - error;
  
//  Serial.print("PWM error Plus/Minus: ");
//  Serial.print(pwmPlus);
//  Serial.print(" ");
//  Serial.println(pwmMinus);
  
  if(pwmPlus > mapSpeed){
     pwmPlus = mapSpeed; 
  }
  else if(pwmPlus < (-1*solveSpeed)){
     pwmPlus = -1 * solveSpeed; 
  }
  
  if(pwmMinus > mapSpeed){
     pwmMinus = mapSpeed; 
  }
  else if(pwmMinus < (-1 * solveSpeed)){
     pwmMinus = -1 * solveSpeed; 
  }
  
  if (pwmRate >= 0){
      if (pwmPlus >= 0){
         rightForward(pwmPlus); 
      }
      else{
         pwmPlus *= -1;
         rightBackward(pwmMinus); 
      }
      if (pwmMinus >= 0){
         leftForward(pwmMinus); 
      }
      else{
         pwmMinus *= -1;
         leftBackward(pwmMinus); 
      }
  }

  else{
    if(pwmPlus >= 0){
       leftBackward(pwmPlus); 
    }
    else{
       pwmPlus *= -1;
       leftForward(pwmPlus); 
    }
    if(pwmMinus >= 0){
       rightBackward(pwmMinus); 
    }
    else{
       pwmMinus *= -1;
       rightForward(pwmMinus); 
    }
  }  

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Error Functions
//Search Term: ERRME
void sysStick(){
 Time = micros();
 deltaTime = Time - prevTime;
 if(deltaTime < 1000) delayMicroseconds(1000-deltaTime);
 prevTime = Time;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Get Sensor Functions
//Search Term: SENME
//Left
double getIRLeft(){
  int recRead = 0;
  digitalWrite(leftEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(leftRecIR);
  digitalWrite(leftEmitIR, LOW);
  if (recRead > 200){
    digitalWrite(leftLED, HIGH);
    wallLeft = true;
  }
  else{
    digitalWrite(leftLED, LOW);
    wallLeft = false;
  }
  return recRead; 
}

//Left Diag
double getIRLeftDiag(){
  int recRead = 0;
  digitalWrite(leftDiagEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(leftDiagRecIR);
  digitalWrite(leftDiagEmitIR, LOW);
  if (recRead > 200){
    digitalWrite(leftDiagLED, HIGH);
    wallLeftDiag = true;
  }
  else{
    digitalWrite(leftDiagLED, LOW);
    wallLeftDiag = false;
  }
  return recRead; 
}

//Left Front
double getIRLeftFront(){
  int recRead = 0;
  digitalWrite(leftFrontEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(leftFrontRecIR);
  digitalWrite(leftFrontEmitIR, LOW);
  if (recRead > 200){
    digitalWrite(leftFrontLED, HIGH);
    wallLeftFront = true;
  }
  else{
    digitalWrite(leftFrontLED, LOW);
    wallLeftFront = false;
  }
  return recRead;
}

//Right Front
double getIRRightFront(){
  int recRead = 0;
  digitalWrite(rightFrontEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(rightFrontRecIR);
  digitalWrite(rightFrontEmitIR, LOW);
  if (recRead > 800){
    digitalWrite(rightFrontLED, HIGH);
    wallRightFront = true;
  }
  else{
    digitalWrite(rightFrontLED, LOW);
    wallRightFront = false;
  }
  return recRead;
}

//Right Diag
double getIRRightDiag(){
  int recRead = 0;
  digitalWrite(rightDiagEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(rightDiagRecIR);
  digitalWrite(rightDiagEmitIR, LOW);
  if (recRead > 200){
    digitalWrite(rightDiagLED, HIGH);
    wallRightDiag = true;
  }
  else{
    digitalWrite(rightDiagLED, LOW);
    wallRightDiag = false;
  }
  return recRead;
}

//Right
double getIRRight(){
  int recRead = 0;
  digitalWrite(rightEmitIR, HIGH );
  delayMicroseconds(80);
  recRead = analogRead(rightRecIR);
  digitalWrite(rightEmitIR, LOW);
  if (recRead > 100){
    digitalWrite(rightLED, HIGH);
    wallRight = true;
  }
  else{
    digitalWrite(rightLED, LOW);
    wallRight = false;
  }
  return recRead;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor Functions
//MOTORME
void leftForward(byte speedIn){
  analogWrite(L_Mtr_A, 0);
  analogWrite(L_Mtr_B,speedIn);
}

void leftBackward(byte speedIn){
  analogWrite(L_Mtr_A,speedIn);
  analogWrite(L_Mtr_B,0);
}

void rightForward(byte speedIn){
  analogWrite(R_Mtr_A, 0);
  analogWrite(R_Mtr_B,speedIn);
}

void rightBackward(byte speedIn){
  analogWrite(R_Mtr_A,speedIn);
  analogWrite(R_Mtr_B,0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Turn Functions
//TURNME
void turn(byte thisDirection){
  switch (thisDirection){
     case LEFT:
       turnLeft(); break;
     case STRAIGHT:
       break;
     case RIGHT:
       turnRight(); break;
     case UTURN:
       turnRight(); turnRight(); break;
     default: break;    
  }
}


// *******************NEED WORK ********************
void turnRight(){
  encTickL = 0;
  encTickR = 0;
  error = 0;
  
  
  while(encTickR < ticksForTurn){
    pwmRate = turnControl();
  
    if(pwmRate > mapSpeed){
       pwmRate = mapSpeed; 
     }
    else if(pwmRate < (-1*solveSpeed)){
      pwmRate = -1 * solveSpeed; 
    }
    
    if (pwmRate >= 0){
       rightBackward(pwmRate);
       leftForward(pwmRate);   
    }
    else{
       rightForward(pwmRate);
       leftBackward(pwmRate);
    }
  }
  
  orientation++;
}

void turnLeft(){
  encTickL = 0;
  encTickR = 0;
  error = 0;
  
  while(encTickR < ticksForTurn){
    pwmRate = turnControl();
  
    if(pwmRate > mapSpeed){
       pwmRate = mapSpeed; 
     }
    else if(pwmRate < (-1*solveSpeed)){
      pwmRate = -1 * solveSpeed; 
    }
    
    if (pwmRate >= 0){
       leftBackward(pwmRate);
       rightForward(pwmRate);   
    }
    else{
       leftForward(pwmRate);
       rightBackward(pwmRate);
    }
  }
  
  orientation--;
}


void checkSensors() {
  getIRLeft();
  getIRLeftFront();
  getIRRight(); 
}

void ImTheMap() {
  floodFillMap[0][15] = floodFillNum;
  wallMap[1][31] = 0;
  dx = 0;
  dy = 1;
}

void updateAllMaps() {
 // updateFloodFillMap();
//  updateWallMap();
}

//void ImTheMap() {
//  floodMap[0][15] = floodfill;
//  wallMap[1][31] = 0;
//  dx = 0;
//  dy = 1;
//}

int absVal(int val) {
  if ( val < 0 )
    val = -val;
  return val;
}

void floodFill(){

  if(FFval < mazeMap[x][y]) FFval = mazeMap[x][y];
  
  //NORTH
  switch(orientation % 4){
    case NORTH :
    if(!wallFront){
      if(FFval < mazeMap[x][y+1]) FFval = mazeMap[x][y+1] - 1; // *********** 1. If our current FFval is LESS than straight, set our current val to the val straight - 1.
    }
    if(!wallRight){
      if(FFval < mazeMap[x+1][y]) FFval = mazeMap[x+1][y] - 1; // ********** 2. if our current FFval is LESS than then right, set our value to the right val - 1.
    }
    if(!wallLeft){
      if(FFval < mazeMap[x-1][y]) FFval = mazeMap[x-1][y] - 1; // ********** 3
    }
    //FF is now right value of what it ought to be
    
    //TURN CASES
    if(wallLeft){ //wall left 
      if(wallRight){ //wall left + wall right
        if(wallFront){ //wall left + wall right + wall front         
          nextTurn = UTURN;
          mazeMap[x][y] = FFval;
          FFval--;
          y--;  
        }
        else{ //wall left + wall right + no wall front.
          nextTurn = STRAIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          y++;
        }
      }
      else{ //wall to the left + no wall to the right        
        if(wallFront){ //wall left + wall front + no wall to the right.
          nextTurn = RIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          x++;
        }
        else{ //left wall + no wall front + no wall right. 
          if(mazeMap[x][y+1] <= mazeMap[x+1][y]){ //The front LESS than the right
            if(mazeMap[x][y+1] <= mazeMap[x][y-1]){ //The front LESS than the right and front less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;           
            }
            else{ //the front less than the right and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
          else{ //the front is not less than the right
            if(mazeMap[x+1][y] <= mazeMap[x][y-1]){ //the front is not less than the right and the right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{ //the front is not less than the right and the righ is NOT less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
        }
      }
    }
    else{ //no wall to the left      
      if(wallRight){ //wall right + no left wall
        if(wallFront){ //wall right + wall front + no left
          nextTurn = LEFT;
          mazeMap[x][y] = FFval;
          FFval--;
          x--;
        }
        else{ //right wall + no wall front + no wall left
          if(mazeMap[x][y+1] <= mazeMap[x-1][y]){ //front is less than left
            if(mazeMap[x][y+1] <= mazeMap[x][y-1]){ //front is less than left and front is less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;           
            }
            else{ //front is less than left and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
          else{ //front is not less than the left
            if(mazeMap[x-1][y] <= mazeMap[x][y-1]){ //front is not less than the left and left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{ //front is not less than the left and left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
        }
      }
      else{ //no wall to the right + no left wall
        if(wallFront){ //no wall right + no wall left + wall front.
          if(mazeMap[x+1][y] <= mazeMap[x-1][y]){ //right is less than the left
            if(mazeMap[x+1][y] <= mazeMap[x][y-1]){ //right is less than the left and right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{//the right is less than the left and the right is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
          else{ //right is not less than the left
            if(mazeMap[x-1][y] <= mazeMap[x][y-1]){ //right is not less than the left and the left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{ //right is not less than the left and the left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
        }
        else{ //no wall to the right + no left wall + no front wall
          if(mazeMap[x][y+1] <= mazeMap[x+1][y]){ //the front is less than the right
            if(mazeMap[x][y+1] <= mazeMap[x-1][y]){ //the front is less than the right and the front is less than the left.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{ //the front is less than the right and the front is not less than the left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
          else{ //the front is not less than right.
            if(mazeMap[x+1][y] <= mazeMap[x-1][y]){ //front is not less than right value and right is less than left.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{ //the front is not less than right and the right is not less than left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
        }
      }
    }
    break;
    case EAST :
    if(!wallFront){
      if(FFval < mazeMap[x+1][y]) FFval = mazeMap[x+1][y] - 1; // *********** 1. If our current FFval is LESS than straight, set our current val to the val straight - 1.
    }
    if(!wallRight){
      if(FFval < mazeMap[x][y-1]) FFval = mazeMap[x][y-1] - 1; // ********** 2. if our current FFval is LESS than then right, set our value to the right val - 1.
    }
    if(!wallLeft){
      if(FFval < mazeMap[x][y+1]) FFval = mazeMap[x][y+1] - 1; // ********** 3
    }
    
    if(wallLeft){ //wall left 
      if(wallRight){ //wall left + wall right
        if(wallFront){ //wall left + wall right + wall front         
          nextTurn = UTURN;
          mazeMap[x][y] = FFval;
          FFval--;
          x--;  
        }
        else{ //wall left + wall right + no wall front.
          nextTurn = STRAIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          x++;
        }
      }
      else{ //wall to the left + no wall to the right        
        if(wallFront){ //wall left + wall front + no wall to the right.
          nextTurn = RIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          y--;
        }
        else{ //left wall + no wall front + no wall right. 
          if(mazeMap[x+1][y] <= mazeMap[x][y-1]){ //The front LESS than the right
            if(mazeMap[x+1][y] <= mazeMap[x-1][y]){ //The front LESS than the right and front less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;           
            }
            else{ //the front less than the right and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
          else{ //the front is not less than the right
            if(mazeMap[x][y-1] <= mazeMap[x-1][y]){ //the front is not less than the right and the right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{ //the front is not less than the right and the righ is NOT less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
        }
      }
    }
    else{ //no wall to the left      
      if(wallRight){ //wall right + no left wall
        if(wallFront){ //wall right + wall front + no left
          nextTurn = LEFT;
          mazeMap[x][y] = FFval;
          FFval--;
          y++;
        }
        else{ //right wall + no wall front + no wall left
          if(mazeMap[x+1][y] <= mazeMap[x][y+1]){ //front is less than left
            if(mazeMap[x+1][y] <= mazeMap[x-1][y]){ //front is less than left and front is less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;           
            }
            else{ //front is less than left and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
          else{ //front is not less than the left
            if(mazeMap[x][y+1] <= mazeMap[x-1][y]){ //front is not less than the left and left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{ //front is not less than the left and left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
        }
      }
      else{ //no wall to the right + no left wall
        if(wallFront){ //no wall right + no wall left + wall front.
          if(mazeMap[x][y-1] <= mazeMap[x][y+1]){ //right is less than the left
            if(mazeMap[x][y-1] <= mazeMap[x-1][y]){ //right is less than the left and right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{//the right is less than the left and the right is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
          else{ //right is not less than the left
            if(mazeMap[x][y+1] <= mazeMap[x-1][y]){ //right is not less than the left and the left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{ //right is not less than the left and the left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
          }
        }
        else{ //no wall to the right + no left wall + no front wall
          if(mazeMap[x+1][y] <= mazeMap[x][y-1]){ //the front is less than the right
            if(mazeMap[x+1][y] <= mazeMap[x][y+1]){ //the front is less than the right and the front is less than the left.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{ //the front is less than the right and the front is not less than the left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
          else{ //the front is not less than right.
            if(mazeMap[x][y-1] <= mazeMap[x][y+1]){ //front is not less than right value and right is less than left.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{ //the front is not less than right and the right is not less than left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
        }
      }
    }
    break;
    case SOUTH :
    if(!wallFront){
      if(FFval < mazeMap[x][y-1]) FFval = mazeMap[x][y-1] - 1; // *********** 1. If our current FFval is LESS than straight, set our current val to the val straight - 1.
    }
    if(!wallRight){
      if(FFval < mazeMap[x-1][y]) FFval = mazeMap[x-1][y] - 1; // ********** 2. if our current FFval is LESS than then right, set our value to the right val - 1.
    }
    if(!wallLeft){
      if(FFval < mazeMap[x+1][y]) FFval = mazeMap[x+1][y] - 1; // ********** 3
    }
    
    if(wallLeft){ //wall left 
      if(wallRight){ //wall left + wall right
        if(wallFront){ //wall left + wall right + wall front         
          nextTurn = UTURN;
          mazeMap[x][y] = FFval;
          FFval--;
          y++;  
        }
        else{ //wall left + wall right + no wall front.
          nextTurn = STRAIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          y--;
        }
      }
      else{ //wall to the left + no wall to the right        
        if(wallFront){ //wall left + wall front + no wall to the right.
          nextTurn = RIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          x--;
        }
        else{ //left wall + no wall front + no wall right. 
          if(mazeMap[x][y-1] <= mazeMap[x-1][y]){ //The front LESS than the right
            if(mazeMap[x][y-1] <= mazeMap[x][y+1]){ //The front LESS than the right and front less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;           
            }
            else{ //the front less than the right and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
          else{ //the front is not less than the right
            if(mazeMap[x-1][y] <= mazeMap[x][y+1]){ //the front is not less than the right and the right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{ //the front is not less than the right and the righ is NOT less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
        }
      }
    }
    else{ //no wall to the left      
      if(wallRight){ //wall right + no left wall
        if(wallFront){ //wall right + wall front + no left
          nextTurn = LEFT;
          mazeMap[x][y] = FFval;
          FFval--;
          x++;
        }
        else{ //right wall + no wall front + no wall left
          if(mazeMap[x][y-1] <= mazeMap[x+1][y]){ //front is less than left
            if(mazeMap[x][y-1] <= mazeMap[x][y+1]){ //front is less than left and front is less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;           
            }
            else{ //front is less than left and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
          else{ //front is not less than the left
            if(mazeMap[x+1][y] <= mazeMap[x][y+1]){ //front is not less than the left and left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{ //front is not less than the left and left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
        }
      }
      else{ //no wall to the right + no left wall
        if(wallFront){ //no wall right + no wall left + wall front.
          if(mazeMap[x-1][y] <= mazeMap[x+1][y]){ //right is less than the left
            if(mazeMap[x-1][y] <= mazeMap[x][y+1]){ //right is less than the left and right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{//the right is less than the left and the right is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
          else{ //right is not less than the left
            if(mazeMap[x+1][y] <= mazeMap[x][y+1]){ //right is not less than the left and the left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
            else{ //right is not less than the left and the left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
          }
        }
        else{ //no wall to the right + no left wall + no front wall
          if(mazeMap[x][y-1] <= mazeMap[x-1][y]){ //the front is less than the right
            if(mazeMap[x][y-1] <= mazeMap[x+1][y]){ //the front is less than the right and the front is less than the left.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{ //the front is less than the right and the front is not less than the left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
          else{ //the front is not less than right.
            if(mazeMap[x-1][y] <= mazeMap[x+1][y]){ //front is not less than right value and right is less than left.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{ //the front is not less than right and the right is not less than left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
        }
      }
    }
    break;
    case WEST :
    if(!wallFront){
      if(FFval < mazeMap[x-1][y]) FFval = mazeMap[x-1][y] - 1; // *********** 1. If our current FFval is LESS than straight, set our current val to the val straight - 1.
    }
    if(!wallRight){
      if(FFval < mazeMap[x][y+1]) FFval = mazeMap[x][y+1] - 1; // ********** 2. if our current FFval is LESS than then right, set our value to the right val - 1.
    }
    if(!wallLeft){
      if(FFval < mazeMap[x][y-1]) FFval = mazeMap[x][y-1] - 1; // ********** 3
    }
    
    if(wallLeft){ //wall left 
      if(wallRight){ //wall left + wall right
        if(wallFront){ //wall left + wall right + wall front         
          nextTurn = UTURN;
          mazeMap[x][y] = FFval;
          FFval--;
          x++;  
        }
        else{ //wall left + wall right + no wall front.
          nextTurn = STRAIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          x--;
        }
      }
      else{ //wall to the left + no wall to the right        
        if(wallFront){ //wall left + wall front + no wall to the right.
          nextTurn = RIGHT;
          mazeMap[x][y] = FFval;
          FFval--;
          y++;
        }
        else{ //left wall + no wall front + no wall right. 
          if(mazeMap[x-1][y] <= mazeMap[x][y+1]){ //The front LESS than the right
            if(mazeMap[x-1][y] <= mazeMap[x+1][y]){ //The front LESS than the right and front less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;           
            }
            else{ //the front less than the right and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
          else{ //the front is not less than the right
            if(mazeMap[x][y+1] <= mazeMap[x+1][y]){ //the front is not less than the right and the right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{ //the front is not less than the right and the righ is NOT less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
        }
      }
    }
    else{ //no wall to the left      
      if(wallRight){ //wall right + no left wall
        if(wallFront){ //wall right + wall front + no left
          nextTurn = LEFT;
          mazeMap[x][y] = FFval;
          FFval--;
          y--;
        }
        else{ //right wall + no wall front + no wall left
          if(mazeMap[x-1][y] <= mazeMap[x][y-1]){ //front is less than left
            if(mazeMap[x-1][y] <= mazeMap[x+1][y]){ //front is less than left and front is less than behind.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;           
            }
            else{ //front is less than left and front is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
          else{ //front is not less than the left
            if(mazeMap[x][y-1] <= mazeMap[x+1][y]){ //front is not less than the left and left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{ //front is not less than the left and left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
        }
      }
      else{ //no wall to the right + no left wall
        if(wallFront){ //no wall right + no wall left + wall front.
          if(mazeMap[x][y+1] <= mazeMap[x][y-1]){ //right is less than the left
            if(mazeMap[x][y+1] <= mazeMap[x+1][y]){ //right is less than the left and right is less than behind.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{//the right is less than the left and the right is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
          else{ //right is not less than the left
            if(mazeMap[x][y-1] <= mazeMap[x+1][y]){ //right is not less than the left and the left is less than behind.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
            else{ //right is not less than the left and the left is not less than behind.
              nextTurn = UTURN;
              mazeMap[x][y] = FFval;
              FFval--;
              x++;
            }
          }
        }
        else{ //no wall to the right + no left wall + no front wall
          if(mazeMap[x-1][y] <= mazeMap[x][y+1]){ //the front is less than the right
            if(mazeMap[x-1][y] <= mazeMap[x][y-1]){ //the front is less than the right and the front is less than the left.
              nextTurn = STRAIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              x--;
            }
            else{ //the front is less than the right and the front is not less than the left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
          else{ //the front is not less than right.
            if(mazeMap[x][y+1] <= mazeMap[x][y-1]){ //front is not less than right value and right is less than left.
              nextTurn = RIGHT;
              mazeMap[x][y] = FFval;
              FFval--;
              y++;
            }
            else{ //the front is not less than right and the right is not less than left.
              nextTurn = LEFT;
              mazeMap[x][y] = FFval;
              FFval--;
              y--;
            }
          }
        }
      }
    }
    break;
    default : 
    break;
  } 
}
