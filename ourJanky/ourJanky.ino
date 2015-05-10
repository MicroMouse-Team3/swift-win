
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

// 0= S, 1 = R, 2 = L, 3 = U
#define LEFT 0
#define STRAIGHT 1
#define RIGHT 2
#define UTURN 3

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//stack<byte> pathStack;
//stack<byte> wallStack;
int navDir;
int dx = 0, dy = 1;
int wallX = 1, wallY = 31;
//Initializations
//INITME
//For Mapping
int currentDirection = 4000;
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
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
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
  
  //Start Up
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
  
  //Determine Inital Offset

  lastTime = micros();
  wallLeftDist = getIRLeft();
  wallRightDist = getIRRight();
  ourOffset = wallRightDist - wallLeftDist;
}

void loop(){
    rightForward(45);
    leftForward(35);
    
     while(encTickL < cellDistance-275){
        jankyPID();
     }
     
     rightBackward(85);
     leftBackward(90);
     while(encTickL < cellDistance){}
     rightBackward(0);
     leftBackward(0);
     delay(1000);
     encTickL = 0;
     encTickR = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID Function
//Search Term: PIDME
void jankyPID(){
  int left = getIRLeft();
  int right = getIRRight();
  int myError = 0;
  
   if (wallLeft && wallRight){
     myError = right - left - ourOffset;
   }
   else if (wallLeft){
     myError = wallLeftDist - left;
   }
   else if (wallRight){
     myError = right - wallRightDist; 
   }
   else{
     myError = 0; 
   }
   
   if (myError < 0){
      leftForward(40);
      rightForward(40); 
   }
   else if (myError > 0){
       leftForward(30);
       rightForward(50);
   }
   else{
      leftForward(35);
      rightForward(45); 
   }
   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Encoder Tick Functions
//Search Term: TICKME
void incEncoderL() {
  encTickL++; 
}
void incEncoderR() {
  encTickR++;
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
//Error Functions
//Search Term: ERRME
void sysStick(){
 Time = micros();
 deltaTime = Time - prevTime;
 if(deltaTime < 1000) delayMicroseconds(1000-deltaTime);
 prevTime = Time;
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

void checkSensors() {
  getIRLeft();
  getIRLeftFront();
  getIRRight(); 
}

void turnLeft(){

}

void turnRight(){

}




