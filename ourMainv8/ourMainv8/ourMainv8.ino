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

#define MAXSIZE 1000

class Stack {
  int stk[MAXSIZE];
  int nelems;
} stack s;

typedef struct Stack Stack;

void Stack_Init(Stack *S) {
  S->nelems = 0;
}

int top(Stack *S) {
  if (S->nelems == 0)
    return -1;
  return S->stk[S->nelems-1];
}
void push(Stack *S, int d) {
  if (S->nelems < MAXSIZE)
    S->data[S->nelems++] = d;
  else
    printf("ERR\n");
}
int pop(Stack *S) {
  if ( S->nelems == 0 )
    printf("ERR\n");
  else
    S->nelems--;
}



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
// 4= S, 5 = R, 6 = L, 7 = U
#define LEFTTURN 4
#define STRAIGHT 5
#define RIGHTTURN 6
#define UTURN 7

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//stack<byte> pathStack;
//stack<byte> wallStack;
Stack pathStack;
Stack wallStack;
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
int mapSpeed = 100;
int solveSpeed = 255; 
int ticksForTurn = 748;

//For PID

// PID gains
double Tkp = 1860L;  // 1.86 * 1000
double Tkd = 86000000L;  // 0.086 * 1000 * 1000000

// Error Tracking
double errOld = 0;
double error = 0;
double currTime = 0;
double lastTime =0;
double delayTime = 0;
int nextTurn = 0;
double deltaTime = 0;

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
                       
int floodFillMap[16][16] =  { -1, };                       
                       
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
  
  //ImTheMap();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Loop Functions
//Search Term: LOOPME
void loop(){
 
//  while(encTickL < cellDistance/2 + setVal){
//    pwmRate = speedControl();
//    PID(pwmRate);
//  }
//  
//  nextTurn = NAV();
//  
//  if(nextTurn == STRAIGHT){
//     setPoint += cellDistance; 
//  }
//
//  if(wallLeftFront){
//    errOld = 0;
//    while(!wallRightFront){
//      pwmRate = wallControl();
//      wallPID(pwmRate); 
//    }
//  }
//  else{
//    while(encTickL < cellDistance + setVal){
//      pwmRate = speedControl();
//      PID(pwmRate); 
//    }
//  }
//  
//  //Map isn't ready yet.
//  //nextTurn = MAP();
//  
//  if (nextTurn != STRAIGHT){
//     setPoint = cellDistance;
//     setVal = 0;
//     encTickL = 0;
//     encTickR = 0; 
//  }
//  else{
//     setVal += cellDistance;
//  }
//  
//  turn(nextTurn); 
//  
  Serial.print("L R: "); Serial.println(getIRLeftFront());
  Serial.print("CR: "); Serial.println(getIRRightFront());
  delay(1000);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Speed Control Function
//Search Term: SPEEDME
int speedControl(){
  errOld = error;
  error = cellDistance - ((encTickR + encTickL)/2);
  pwmRate = Tkp * error;
  currTime = micros();
  deltaTime = currTime - lastTime;
  pwmRate += Tkd * (error - errOld)/deltaTime;
  pwmRate /= 1000;
  lastTime = currTime;
  return pwmRate;
}

int wallControl(){
  //Needs to be able to stop when very close to wall.
  
  int distanceToStop = 500; //random value to be used. This will become how close we want to stop in front of the wall.
  int currentDistance = getIRRightFront();
  
  errOld = error;
  error = distanceToStop - currentDistance;
  pwmRate = Tkp * error;
  currTime = micros();
  deltaTime = currTime - lastTime;
  pwmRate += Tkd * (error - errOld)/deltaTime;
  pwmRate /= 1000;
  lastTime = currTime;
  return pwmRate;
}

int turnControl(){
  //Needs to be able to stop when very close to wall.
  
  errOld = error;
  error = ticksForTurn - ((encTickR + encTickL)/2);
  pwmRate = Tkp * error;
  currTime = micros();
  deltaTime = currTime - lastTime;
  pwmRate += Tkd * (error - errOld)/deltaTime;
  pwmRate /= 1000;
  lastTime = currTime;
  return pwmRate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID Function
//Search Term: PIDME
void PID(int PWMRate){
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
     errorP = 2*(wallLeftDist - left);
     errorD = errorP - oldErrorP;
  }
  
  else if(right){
     errorP = 2*(right - wallRightDist);
     errorD = errorP - oldErrorP;
  }
  
  else if (!left && !right){
     errorP = 0;
     errorD = 0; 
  }
  
  error = (KP * errorP) + (KD * errorD);
  oldErrorP = errorP;
  
  pwmPlus = pwmRate + error;
  pwmMinus = pwmRate - error;
  
  if(pwmPlus > mapSpeed){
     pwmPlus = mapSpeed; 
  }
  else if(pwmPlus < (-1*solveSpeed)){
     pwmPlus = -1 * solveSpeed; 
  }
  
  if(pwmMinus > (-1 * solveSpeed)){
     pwmMinus = -1*solveSpeed; 
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

// ************************NEEDS WORK************************************
void wallPID(int PWMRate){
  int error = 0;
  int errorP = 0;
  int errorD = 0;
  static int oldErrorP = 0;
  int pwmPlus = 0;
  int pwmMinus = 0;
  unsigned int KP = 1;
  unsigned int KD = 1;
  
  int left = getIRLeftDiag();
  int right = getIRRightDiag();
  
  if (wallLeft && wallRight){
     errorP = right - left;
     errorD = errorP - oldErrorP;
  }
  
  else if(left){
     errorP = 2*(wallLeftDist - left);
     errorD = errorP - oldErrorP;
  }
  
  else if(right){
     errorP = 2*(right - wallRightDist);
     errorD = errorP - oldErrorP;
  }
  
  else if (!left && !right){
     errorP = 0;
     errorD = 0; 
  }
  
  error = (KP * errorP) + (KD * errorD);
  oldErrorP = errorP;
  
  pwmPlus = pwmRate + error;
  pwmMinus = pwmRate - error;
  
  if(pwmPlus > mapSpeed){
     pwmPlus = mapSpeed; 
  }
  else if(pwmPlus < (-1*solveSpeed)){
     pwmPlus = -1 * solveSpeed; 
  }
  
  if(pwmMinus > (-1 * solveSpeed)){
     pwmMinus = -1*solveSpeed; 
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
  if (recRead > 200){
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
  if (recRead > 200){
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
     case LEFTTURN:
       turnLeft(); break;
     case STRAIGHT:
       break;
     case RIGHTTURN:
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
  
  currentDirection++;
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
  
  currentDirection--;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////pathStack.push(STRAIGHT);
//
////byte floodFill[16][16];
//
//for( byte i = 0 ; i < 256 ; i++ ){
//  for( byte j = 0 ; j < 256 ; j++ ){
//    floodFill[i][j] = -1;
//  }
//}
//  floodFill[0][0] = 256;
//  floodFill[8][8] = 0;
//  floodFill[8][9] = 0;
//  floodFill[9][8] = 0;
//  floodFill[9][9] = 0;


// 4= L, 5 = S, 6 = R, 7 = U



byte mapDisThang( Stack pathStack, Stack nextTurn) {

  if ( floodFillNum < currentFFVal )
    floodFillNum = currentFFVal;
  
   updateWallMap();  
  
  
  floodFill[posX][posY] = floodFillNum--;  
  
  if(wallLeft){
     if(wallRight){
        if(wallFront){          
          //TODO: Fix this
          return UTURN;         
        }
        else{
          //TODO:  conditional of prev floodfillVal
          return STRAIGHT;
        }
     }
    else{ //no wall right
       if(wallFront){
                    
          return RIGHTTURN;
          
       }
       else{
          switch(currentDirection % 4){
            case NORTH : if(floodFillMap[x][y+1] > floodFillMap[x+1][y]){
                           
                           return RIGHTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(floodFillMap[x+1][y] > floodFillMap[x][y-1]){
                           return RIGHTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(floodFillMap[x][y-1] > floodFillMap[x-1][y]){
                           return RIGHTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(floodFillMap[x-1][y] > floodFillMap[x][y+1]){
                           return RIGHTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
            
       }
    } 
  }
  else{ //no wallLeft
    if(wallRight){        
        if(wallLeftFront){
          if ( floodFillNum < floodFillMap[x][y] )
            floodFillNum = floodFillMap[x][y];
            
          floodFillMap[x][y] = floodFillMap[xPrev][yPrev] - 1;
          updateWallMap();
          
          return LEFTTURN;
          
        }
        else{ //no wall front
          switch(currentDirection % 4){
            case NORTH : if(floodFillMap[x][y+1] > floodFillMap[x-1][y]){                               
              
                           return LEFTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(floodFillMap[x+1][y] > floodFillMap[x][y+1]){
                           return LEFTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(floodFillMap[x][y-1] > floodFillMap[x+1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(floodFillMap[x-1][y] > floodFillMap[x][y-1]){
                           return LEFTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
        }
     }
    else{ //No wallRight
       if(wallLeftFront){
          switch(currentDirection % 4){
            case NORTH : if(floodFillMap[x-1][y] < floodFillMap[x+1][y]){
                           return LEFTTURN;
                        }    
                       else{
                         return RIGHTTURN;
                       }
                       break;  
            case EAST : if(floodFillMap[x][y+1] < floodFillMap[x][y-1]){
                           return LEFTTURN;
                        }
                       else{
                         return RIGHTTURN;
                       }
                       break;
            case SOUTH : if(floodFillMap[x-1][y] > floodFillMap[x+1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            case WEST : if(floodFillMap[x][y-1] < floodFillMap[x][y+1]){
                           return LEFTTURN;
                         }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            }   
       }
       else{ //no walls at all, priority is STRAIGHT THEN TURN RIGHT if its equal
       
           switch(nextTurn) {
             case STRAIGHT:
                             switch(currentDirection) {
                               case NORTH:
                                 if ( floodFillMap[x][y+1] == -1 )
                                   floodFillMap[x][y+1] == floodFillMap[xprev][yprev];                                   
                                 else if ( floodFillMap[x-1][y] == -1 )
                                   nextTurn = LEFTTURN;                                                                                                                                      
                                 else if ( floodFillMap[x+1][y] == -1 )
                                   floodFillMap[x+1][y] == floodFillMap[xprev][yprev];                                   
                                 else if ( floodFillMap[x-1][y] == -1 )
                                   nextTurn = RIGHTTURN;
                                 else {
                                   if ( Math.abs(floodFillMap[x+1][y] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y +1] - floodFillNum) == 1 || Math.abs(floodFillMap[x-1][y] - floodFillNum) == 1 ) {
                                     if ( floodFillMap[x+1][y] > floodFillMap[x][y+1] )
                                       if ( floodFillMap[x+1][y] > floodFillMap[x-1][y] )
                                         nextTurn = RIGHTTURN;
                                     else if ( floodFillMap[x-1][y] > floodFillMap[x][y+1] )
                                       nextTurn = LEFTTURN;
                                     else
                                       nextTurn = STRAIGHT;  
                                   }
                                 }                                                                      
                                 break;
                               case EAST:
                                 if ( floodFillMap[x+1][y] == -1 )
                                   floodFillMap[x+1][y] == floodFillMap[xprev][yprev];                                   
                                 else if ( floodFillMap[x][y-1] == -1 )
                                   nextTurn = RIGHTTURN;                                                                                                                                      
                                 else if ( floodFillMap[x][y+1] == -1 )
                                   floodFillMap[x][y+1] == floodFill[xprev][yprev];                                   
                                 else if ( floodFillMap[x][y+1] == -1 )
                                   nextTurn = LEFTTURN;
                                 else {
                                   if ( Math.abs(floodFillMap[x+1][y] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y-1] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y+1] - floodFillNum) == 1 ) {
                                     if ( floodFillMap[x+1][y] > floodFillMap[x][y+1] )
                                       if ( floodFillMap[x+1][y] > floodFillMap[x][y-1] )
                                         nextTurn = STRAIGHT;
                                     else if ( floodFillMap[x][y-1] > floodFillMap[x][y+1] )
                                       nextTurn = RIGHTTURN;
                                     else
                                       nextTurn = LEFTTURN;  
                                   }
                                 }       
                                 break;
                               case SOUTH:
                                 if ( floodFillMap[x][y+1] == -1 )
                                   floodFillMap[x][y+1] == floodFillMap[xprev][yprev];                                   
                                 else if ( floodFillMap[x-1][y] == -1 )
                                   nextTurn = RIGHTTURN;                                                                                                                                      
                                 else if ( floodFillMap[x+1][y] == -1 )
                                   nextTurn = LEFTTURN;
//                                 else if ( floodFillMap[x][y+1] == -1 )
//                                   nextTurn = LEFTTURN;
                                 else {
                                   if ( Math.abs(floodFillMap[x][y+1] - floodFillNum) == 1 || Math.abs(floodFillMap[x-1][y] - floodFillNum) == 1 || Math.abs(floodFillMap[x+1][y] - floodFillNum) == 1 ) {
                                     if ( floodFillMap[x+1][y] > floodFillMap[x][y+1] )
                                       if ( floodFillMap[x+1][y] > floodFillMap[x-1][y] )
                                         nextTurn = LEFTTURN;
                                     else if ( floodFillMap[x-1][y] > floodFillMap[x][y+1] )
                                       nextTurn = RIGHTTURN;
                                     else
                                       nextTurn = STRAIGHT;  
                                   }
                                 }                                      
                                 break;
                               case WEST:
                                 if ( floodFillMap[x-1][y] == -1 )
                                   floodFillMap[x-1][y] == floodFillMap[xprev][yprev];                                   
                                 else if ( floodFillMap[x][y+1] == -1 )
                                   nextTurn = RIGHTTURN;                                                                                                                                      
                                 else if ( floodFillMap[x][y-1] == -1 )
                                   nextTurn = LEFTTURN;
//                                 else if ( floodFillMap[x][y+1] == -1 )
//                                   nextTurn = LEFTTURN;
                                 else {
                                   if ( Math.abs(floodFillMap[x-1][y] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y+1] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y-1] - floodFillNum) == 1 ) {
                                     if ( floodFillMap[x][y-1] > floodFillMap[x-1][y] )
                                       if ( floodFillMap[x][y-1] > floodFillMap[x][y+1] )
                                         nextTurn = LEFTTURN;
                                     else if ( floodFillMap[x][y+1] > floodFillMap[x-1][y] )
                                       nextTurn = RIGHTTURN;
                                     else
                                       nextTurn = STRAIGHT;  
                                   }
                                 }                                                       
                                 break;                            
                             }   break;
             case RIGHTTURN:
             //-Francisco: We need to reconfigure our floodfill values to now store it into FFVAL, previoous code does not do this. The last peice of code in this function might do this, but need for reevaluation to make sure.
                           //reordered the presedence of conditionals to reflect the logic of SwirlIEEEs choices.
                             switch(currentDirection) {
                               case NORTH:
                                 if ( floodFillMap[x+1][y] == -1 )
                                   floodFillNum = floodFillMap[x+1][y];
                                 else if ( floodFillMap[x][y+1] == -1 )
                                   nextTurn = STRAIGHT;                                                                                                                                      
                                 else if(floodFillMap[x-1][y] == -1 )
                                 {
                                   floodFillNum == floodFillMap[x-1][y];                                   
                                   nextTurn = LEFTTURN;
                                 }
                                //this else if is in case every value in adjacent cell does not have a -1
                                 else{
                                   if ( Math.abs(floodFillMap[x+1][y] - floodFillNum) == 1 || Math.abs(floodFillMap[x][y +1] - floodFillNum) == 1 || Math.abs(floodFillMap[x-1][y] - floodFillNum) == 1 ) {
                                     if ( floodFillMap[x+1][y] > floodFillMap[x][y+1] ){
                                       if ( floodFillMap[x+1][y] > floodFillMap[x-1][y] )
                                         nextTurn = RIGHTTURN;
                                     }
                                     else if ( floodFillMap[x-1][y] > floodFillMap[x][y+1] )
                                       nextTurn = LEFTTURN;
                                     else
                                       nextTurn = STRAIGHT;  
                                   }
                                 }
                                 
                                 break;
                                 break;
                               case EAST:
                                 break;
                               case SOUTH:
                                 break;
                               case WEST:
                                 break;                            
                             }   break;
             case LEFTTURN:
                             switch(currentDirection) {
                               case NORTH:
                                 break;
                               case EAST:
                                 break;
                               case SOUTH:
                                 break;
                               case WEST:
                                 break;                            
                             }   break;
             case UTURN:
                             switch(currentDirection) {
                               case NORTH:
                                 break;
                               case EAST:
                                 break;
                               case SOUTH:
                                 break;
                               case WEST:
                                 break;                            
                             }   break;
           }
       
          if ( floodFillNum < floodFillMap[x][y] )
            floodFillNum = floodFillMap[x][y];
          switch(currentDirection % 4){
            case NORTH : if(floodFillMap[x-1][y] < floodFillMap[x+1][y]){
                             if(floodFillMap[x-1][y] < floodFillMap[x][y+1])
                                 return LEFTTURN;
                             else if(floodFillMap[x][y+1] > floodFillMap[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(floodFillMap[x-1][y] < floodFillMap[x+1][y]){
                             if(floodFillMap[x+1][y] > floodFillMap[x][y+1])
                                 return RIGHTTURN;
                             else if(floodFillMap[x][y+1] > floodFillMap[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(floodFillMap[x-1][y] == floodFillMap[x+1][y])
                               return RIGHTTURN;
                            else if(floodFillMap[x][y+1] == floodFillMap[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       /*
                       if ( adjCell > 1 && adjCell > floodfillNum && adjCell != -1 )
                            floodFillNum = adjCell - 1;                       
                         */   
                       if ( floodFillNum[x+1][y] - floodFillNum > 1 && floodFillNum[x+1][y] > floodFillNum && floodFillNum[x+1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                     
                       else if ( floodFillNum[x][y+1] - floodFillNum > 1 && floodFillNum[x][y+1] > floodFillNum && floodFillNum[x][y+1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x-1][y] - floodFillNum > 1 && floodFillNum[x-1][y] > floodFillNum && floodFillNum[x-1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
//                       else if ( floodFillNum[x][y-1] - floodFillNum > 1 && floodFillNum[x][y-1] > floodFillNum && floodFillNum[x][y-1] != -1 )
  //                          floodfillNum = floodFillNum[x+1][y] - 1;                                                                                                       

                        
                       
                       break;  
                       
            case EAST : if(floodFillMap[x][y+1] > floodFillMap[x][y-1]){
                             if(floodFillMap[x][y+1] > floodFillMap[x+1][y])
                                 return LEFTTURN;
                             else if(floodFillMap[x][y+1] > floodFillMap[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(floodFillMap[x][y+1] < floodFillMap[x][y-1]){
                             if(floodFillMap[x][y-1] > floodFillMap[x+1][y])
                                 return RIGHTTURN;
                             else if(floodFillMap[x+1][y] > floodFillMap[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(floodFillMap[x-1][y] == floodFillMap[x+1][y])
                               return RIGHTTURN;
                            else if(floodFillMap[x][y+1] == floodFillMap[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       
                       if ( floodFillNum[x+1][y] - floodFillNum > 1 && floodFillNum[x+1][y] > floodFillNum && floodFillNum[x+1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                     
                       else if ( floodFillNum[x][y+1] - floodFillNum > 1 && floodFillNum[x][y+1] > floodFillNum && floodFillNum[x][y+1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
//                       else if ( floodFillNum[x-1][y] - floodFillNum > 1 && floodFillNum[x-1][y] > floodFillNum && floodFillNum[x-1][y] != -1 )
  //                          floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x][y-1] - floodFillNum > 1 && floodFillNum[x][y-1] > floodFillNum && floodFillNum[x][y-1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                               
                       
                       break;
                       
            case SOUTH : if(floodFillMap[x-1][y] < floodFillMap[x+1][y]){
                             if(floodFillMap[x+1][y] > floodFillMap[x][y-1])
                                 return LEFTTURN;
                             else if(floodFillMap[x][y-1] > floodFillMap[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(floodFillMap[x-1][y] > floodFillMap[x+1][y]){
                             if(floodFillMap[x-1][y] > floodFillMap[x][y-1])
                                 return RIGHTTURN;
                             else if(floodFillMap[x][y-1] > floodFillMap[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(floodFillMap[x-1][y] == floodFillMap[x+1][y])
                               return RIGHTTURN;
                            else if(floodFillMap[x][y+1] == floodFillMap[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       
                       if ( floodFillNum[x+1][y] - floodFillNum > 1 && floodFillNum[x+1][y] > floodFillNum && floodFillNum[x+1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                     
//                       else if ( floodFillNum[x][y+1] - floodFillNum > 1 && floodFillNum[x][y+1] > floodFillNum && floodFillNum[x][y+1] != -1 )
  //                          floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x-1][y] - floodFillNum > 1 && floodFillNum[x-1][y] > floodFillNum && floodFillNum[x-1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x][y-1] - floodFillNum > 1 && floodFillNum[x][y-1] > floodFillNum && floodFillNum[x][y-1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                               
                       
                       break; 
                         
            case WEST : if(floodFillMap[x][y+1] < floodFillMap[x][y-1]){
                             if(floodFillMap[x][y-1] > floodFillMap[x-1][y])
                                 return LEFTTURN;
                             else if(floodFillMap[x-1][y] > floodFillMap[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(floodFillMap[x][y+1] > floodFillMap[x][y-1]){
                             if(floodFillMap[x][y+1] > floodFillMap[x-1][y])
                                 return RIGHTTURN;
                             else if(floodFillMap[x-1][y] > floodFillMap[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(floodFillMap[x-1][y] == floodFillMap[x+1][y])
                               return RIGHTTURN;
                            else if(floodFillMap[x][y+1] == floodFillMap[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       
//                       if ( floodFillNum[x+1][y] - floodFillNum > 1 && floodFillNum[x+1][y] > floodFillNum && floodFillNum[x+1][y] != -1 )
  //                          floodfillNum = floodFillNum[x+1][y] - 1;                     
                       if ( floodFillNum[x][y+1] - floodFillNum > 1 && floodFillNum[x][y+1] > floodFillNum && floodFillNum[x][y+1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x-1][y] - floodFillNum > 1 && floodFillNum[x-1][y] > floodFillNum && floodFillNum[x-1][y] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;   
                       else if ( floodFillNum[x][y-1] - floodFillNum > 1 && floodFillNum[x][y-1] > floodFillNum && floodFillNum[x][y-1] != -1 )
                            floodfillNum = floodFillNum[x+1][y] - 1;                               
                       
                       break;
            }
            
       }
    }
    floodFill[x][y] = floodFillNum--;
    return nextTurn;
  }
  
 return STRAIGHT;
  /*
=======
  /**
  if(navDir == STRAIGHT){
     if(floodFill[x][y] != -1) // check problem for begining start
       floodFill[x][y] = flood[xprev][yprev] -1;
       return navDir; //no need to override
     else if(floodFill[x][y]>floodFill[xprev][yprev]){
         turnAround(); //turn around first and return straight
         return STRAIGHT;
     }
     else if(floodFill[x][y]< floodFill[xprev][yprev]){
         floodfill[x][y] = floodFill[xprev][yprev] - 1;
     }
        
  }
  else if{navDir == RIGHTTURN){
    if(floodFill[x][y] != -1)
       floodFill[x][y] = flood[xprev][yprev] -1;
       return navDir; //no need to override
     else if(floodFill[x][y]>floodFill[xprev][yprev]){
         turnLeft(); //turn left first and return straight
         return STRAIGHT;
     }
     else if(floodFill[x][y]< floodFill[xprev][yprev]){
         floodfill[x][y] = floodFill[xprev][yprev] - 1;
     }
    
  }
  else if(navDir == LEFTTURN){
   
  }
  else{ // UTURN
    
  }
  */
  }
  
  **/
  
  if(wallLeft){
     if(wallRight){
        if(wallFront){          
          /*******TO DO *******/
        }
        else{
          
          
        }
     }
    else{ //no wall right
       if(wallLeftFront){
          return RIGHTTURN;
          
       }
       else{
          switch(currentDirection % 4){
            case NORTH : if(maze[x][y+1] < maze[x+1][y]){
                           return RIGHTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(maze[x+1][y] < maze[x][y-1]){
                           return RIGHTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(maze[x][y-1] < maze[x-1][y]){
                           return RIGHTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(maze[x-1][y] < maze[x][y+1]){
                           return RIGHTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
            
       }
    } 
  }
  else{ //no wallLeft
    if(wallRight){
        if(wallLeftFront){
          
          return LEFTTURN;
          
        }
        else{ //no wall front
          switch(currentDirection % 4){
            case NORTH : if(maze[x][y+1] < maze[x-1][y]){
                           return LEFTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(maze[x+1][y] < maze[x][y+1]){
                           return LEFTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(maze[x][y-1] < maze[x+1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(maze[x-1][y] < maze[x][y-1]){
                           return LEFTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
        }
     }
    else{ //No wallRight
       if(wallLeftFront){
          switch(currentDirection % 4){
            case NORTH : if(maze[x-1][y] > maze[x+1][y]){
                           return LEFTTURN;
                        }    
                       else{
                         return RIGHTTURN;
                       }
                       break;  
            case EAST : if(maze[x][y+1] > maze[x][y-1]){
                           return LEFTTURN;
                        }
                       else{
                         return RIGHTTURN;
                       }
                       break;
            case SOUTH : if(maze[x+1][y] > maze[x-1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            case WEST : if(maze[x][y-1] > maze[x][y+1]){
                           return LEFTTURN;
                         }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            }   
       }
       else{ //no walls at all, priority is STRAIGHT THEN TURN RIGHT if its equal
          switch(currentDirection % 4){
            case NORTH : if(maze[x-1][y] > maze[x+1][y]){
                             if(maze[x-1][y] > maze[x][y+1])
                                 return LEFTTURN;
                             else if(maze[x][y+1] > maze[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x-1][y] < maze[x+1][y]){
                             if(maze[x+1][y] > maze[x][y+1])
                                 return RIGHTTURN;
                             else if(maze[x][y+1] > maze[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       
                       break;  
                       
            case EAST : if(maze[x][y+1] > maze[x][y-1]){
                             if(maze[x][y+1] > maze[x+1][y])
                                 return LEFTTURN;
                             else if(maze[x][y+1] > maze[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x][y+1] < maze[x][y-1]){
                             if(maze[x][y-1] > maze[x+1][y])
                                 return RIGHTTURN;
                             else if(maze[x+1][y] > maze[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break;
                       
            case SOUTH : if(maze[x-1][y] < maze[x+1][y]){
                             if(maze[x+1][y] > maze[x][y-1])
                                 return LEFTTURN;
                             else if(maze[x][y-1] > maze[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x-1][y] > maze[x+1][y]){
                             if(maze[x-1][y] > maze[x][y-1])
                                 return RIGHTTURN;
                             else if(maze[x][y-1] > maze[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break; 
                         
            case WEST : if(maze[x][y+1] < maze[x][y-1]){
                             if(maze[x][y-1] > maze[x-1][y])
                                 return LEFTTURN;
                             else if(maze[x-1][y] > maze[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x][y+1] > maze[x][y-1]){
                             if(maze[x][y+1] > maze[x-1][y])
                                 return RIGHTTURN;
                             else if(maze[x-1][y] > maze[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break;
            }
            
       }
    }
  }
  
 return STRAIGHT;
      
}
//byte mapDisThang(pathStack, navDir) {
//    
//  if(navDir == STRAIGHT){
//     if(floodFill[x][y] != -1)
//       floodFill[x][y] = flood[xprev][yprev] -1;
//       return navDir; //no need to override
//     else if(floodFill[x][y]>floodFill[xprev][yprev]){
//         turnAround(); //turn around first and return straight
//         return STRAIGHT;
//     }
//     else if(floodFill[x][y]< floodFill[xprev][yprev]){
//         floodfill[x][y] = floodFill[xprev][yprev] - 1;
//     }
//        
//  }
//  else if{navDir == RIGHTTURN){
//    if(floodFill[x][y] != -1)
//       floodFill[x][y] = flood[xprev][yprev] -1;
//       return navDir; //no need to override
//     else if(floodFill[x][y]>floodFill[xprev][yprev]){
//         turnLeft(); //turn left first and return straight
//         return STRAIGHT;
//     }
//     else if(floodFill[x][y]< floodFill[xprev][yprev]){
//         floodfill[x][y] = floodFill[xprev][yprev] - 1;
//     }
//    
//  }
//  else if(navDir == LEFTTURN){
//   
//  }
//  else{ // UTURN
//    
//  }
//  
//  }
//  
//}
//NAV Function
//NAVME
byte NAV(){
  
  //0 = N, 1 = E, 2 = S, 3 = W  
  // 4= L, 5 = S, 6 = R, 7 = U
  
  //Determines wallLeft, wallRight, wallFront boolean values

  checkSensors();


  
  if(wallLeft){
     if(wallRight){
        if(wallLeftFront){          
          return UTURN;         
        }
        else{
          return STRAIGHT;
        }
     }
    else{ //no wall right
       if(wallLeftFront){
          return RIGHTTURN;
          
       }
       else{
          switch(currentDirection % 4){
            case NORTH : if(maze[x][y+1] < maze[x+1][y]){
                           return RIGHTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(maze[x+1][y] < maze[x][y-1]){
                           return RIGHTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(maze[x][y-1] < maze[x-1][y]){
                           return RIGHTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(maze[x-1][y] < maze[x][y+1]){
                           return RIGHTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
            
       }
    } 
  }
  else{ //no wallLeft
    if(wallRight){
        if(wallLeftFront){
          
          return LEFTTURN;
          
        }
        else{ //no wall front
          switch(currentDirection % 4){
            case NORTH : if(maze[x][y+1] < maze[x-1][y]){
                           return LEFTTURN;
                        }    
                       else{
                         return STRAIGHT;
                       }
                       break;  
            case EAST : if(maze[x+1][y] < maze[x][y+1]){
                           return LEFTTURN;
                        }
                       else{
                         return STRAIGHT;
                       }
                       break;
            case SOUTH : if(maze[x][y-1] < maze[x+1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return STRAIGHT;
                         }
                         break;
            case WEST : if(maze[x-1][y] < maze[x][y-1]){
                           return LEFTTURN;
                         }
                         else{
                           return STRAIGHT;
                         }
                         break;
            }
        }
     }
    else{ //No wallRight
       if(wallLeftFront){
          switch(currentDirection % 4){
            case NORTH : if(maze[x-1][y] > maze[x+1][y]){
                           return LEFTTURN;
                        }    
                       else{
                         return RIGHTTURN;
                       }
                       break;  
            case EAST : if(maze[x][y+1] > maze[x][y-1]){
                           return LEFTTURN;
                        }
                       else{
                         return RIGHTTURN;
                       }
                       break;
            case SOUTH : if(maze[x+1][y] > maze[x-1][y]){
                           return LEFTTURN;
                          }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            case WEST : if(maze[x][y-1] > maze[x][y+1]){
                           return LEFTTURN;
                         }
                         else{
                           return RIGHTTURN;
                         }
                         break;
            }   
       }
       else{ //no walls at all, priority is STRAIGHT THEN TURN RIGHT if its equal
          switch(currentDirection % 4){
            case NORTH : if(maze[x-1][y] > maze[x+1][y]){
                             if(maze[x-1][y] > maze[x][y+1])
                                 return LEFTTURN;
                             else if(maze[x][y+1] > maze[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x-1][y] < maze[x+1][y]){
                             if(maze[x+1][y] > maze[x][y+1])
                                 return RIGHTTURN;
                             else if(maze[x][y+1] > maze[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break;  
                       
            case EAST : if(maze[x][y+1] > maze[x][y-1]){
                             if(maze[x][y+1] > maze[x+1][y])
                                 return LEFTTURN;
                             else if(maze[x][y+1] > maze[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x][y+1] < maze[x][y-1]){
                             if(maze[x][y-1] > maze[x+1][y])
                                 return RIGHTTURN;
                             else if(maze[x+1][y] > maze[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break;
                       
            case SOUTH : if(maze[x-1][y] < maze[x+1][y]){
                             if(maze[x+1][y] > maze[x][y-1])
                                 return LEFTTURN;
                             else if(maze[x][y-1] > maze[x-1][y])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x-1][y] > maze[x+1][y]){
                             if(maze[x-1][y] > maze[x][y-1])
                                 return RIGHTTURN;
                             else if(maze[x][y-1] > maze[x+1][y])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break; 
                         
            case WEST : if(maze[x][y+1] < maze[x][y-1]){
                             if(maze[x][y-1] > maze[x-1][y])
                                 return LEFTTURN;
                             else if(maze[x-1][y] > maze[x][y+1])
                                 return STRAIGHT;
                             else 
                                 return RIGHTTURN;
                         }    
                       else if(maze[x][y+1] > maze[x][y-1]){
                             if(maze[x][y+1] > maze[x-1][y])
                                 return RIGHTTURN;
                             else if(maze[x-1][y] > maze[x][y-1])
                                 return STRAIGHT;
                             else 
                                 return LEFTTURN;
                       }
                       else{
                            if(maze[x-1][y] == maze[x+1][y])
                               return RIGHTTURN;
                            else if(maze[x][y+1] == maze[x+1][y])
                               return STRAIGHT;
                            else
                               return STRAIGHT;
                       }
                       break;
            }
            
       }
    }
  }
  
 return STRAIGHT;      
}




//byte MAPME(){
//  checkSensors();
//  mapTurn(nextTurn);
//  updateWalls();
//  return STRAIGHT;  
//}



// This function updates the floodfill value
void updateMap(){
    
}

void updateFloodMap() {
  
}

//void updateMap(int nextTurn){
//	if (nextTurn == STRAIGHT){ //STRAIGHT
//		mapTurn(nextTurn);
//		floodMap[posX+dx][posY+dy] = floodfill--;
//	}
//	if (nextTurn == LEFTTURN){
//		mapTurn(nextTurn);
//		floodMap[posX+dx][posY+dy] = floodfill--;
//	}
//	if(nextTurn == RIGHTTURN){
//		mapTurn(nextTurn);
//		floodMap[posX+dx][posY+dy] = floodfill--;
//	}
//	if(nextTurn == UTURN){
//		mapTurn(nextTurn);
//		floodMap[posX+dx][posY+dy] = floodfill--;
//	}
//}

// this function updates the wall locations and places known to have no walls
void updateWalls(){
  //update behind us
  
  /*
  wallMap[wallX-dx][wallY-dy] = 0;
  
  //wall on left
  if ( wallLeft )
    wallMap[wallX-1][wallY] = 1;  
  else
    wallMap[wallX-1][wallY] = 0;
    
  //wall on right
  if ( wallRight )
    wallMap[wallX+1][wallY] = 1;
  else
    wallMap[wallX+1][wallY] = 0;  
  
  //wall in front
  if ( wallFront )
    wallMap[wallX][wallY+1] = 1;  
  else
    wallMap[wallX][wallY+1] = 0;  

  wallX = wallX + 2 * dx;
  wallY = wallY + 2 * dy;
  */
}

// This function solves the flood fill
void SOLVE(){
	
}

// This function drives straight to the finish
void solve(){
  
}

// This function returns to the starting position
void start(){
  
}


void mapTurn( int nextTurn ) {
  int tmp = dx;
  if ( nextTurn == LEFTTURN ) {
    dx = -dy;
    dy = tmp;
  } else if ( nextTurn == RIGHTTURN ) {
    dx = dy;
    dy = -tmp;
  } else if ( nextTurn == UTURN ) {
  if ( nextTurn == LEFTTURN ) {
    dx = -dy;
    dy = tmp;
  } else if ( nextTurn == RIGHTTURN ) {
    dx = dy;
    dy = -tmp;
  } else if ( nextTurn == UTURN ) {
    dx = -dx;
    dy = -dy;
  }
  //else straight keeps same parameters
  if ( nextTurn == currentDirection ) {
    dx = -dy;
    dy = tmp;
  } else if ( nextTurn == currentDirection ) {
    dx = dy;
    dy = -tmp;
  } else if ( nextTurn == currentDirection ) {
    dx = -dx;
    dy = -dy;
  }   
}
}

//void mapTurn( int nextTurn ) {
//  int tmp = dx;
//  if ( nextTurn == LEFTTURN ) {
//    dx = -dy;
//    dy = tmp;
//  } else if ( nextTurn == RIGHTTURN ) {
//    dx = dy;
//    dy = -tmp;
//  } else if ( nextTurn == UTURN ) {
//  if ( nextTurn == LEFTTURN ) {
//    dx = -dy;
//    dy = tmp;
//  } else if ( nextTurn == RIGHTTURN ) {
//    dx = dy;
//    dy = -tmp;
//  } else if ( nextTurn == UTURN ) {
//    dx = -dx;
//    dy = -dy;
//  }
//  //else straight keeps same parameters
//  if ( nextTurn == currentDirection ) {
//    dx = -dy;
//    dy = tmp;
//  } else if ( nextTurn == currentDirection ) {
//    dx = dy;
//    dy = -tmp;
//  } else if ( nextTurn == currentDirection ) {
//    dx = -dx;
//    dy = -dy;
//  }   
//}



void checkSensors() {
  wallLeft = getIRLeft() > 600;
  wallFront = wallLeftFront = getIRLeftFront() > 200;
  wallRight = getIRRight() > 600; 
}

void ImTheMap() {
  floodMap[0][15] = floodfill;
  wallMap[1][31] = 0;
  dx = 0;
  dy = 1;
}

void updateAllMaps() {
  updateFloodFillMap();
  updateWallMap();
}



/*

void push( int input ) {
  int num;
  if (s.top == (MAXSIZE - 1) ) {
    printf("Stack is Full\n");
    return;
  } else {
    s.top++;
    s.stk[s.top] = input;
  }
  return;
}

int pop() {
  int num;
  if ( s.top == -1 ) {
    printf("Stack is empty\n");
    return (s.top);
  } else {
    num = s.stk[s.top];
    s.top--;
  }
  return (num);
}




*/
