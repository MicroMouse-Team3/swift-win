
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
//Initializations
//INITME

//For Mapping
int orientation = 4000;
int x = 0;
int y = 0;

bool solved = false;

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
int ticksForTurn = 975;
int uTicks = 1625;
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

int mazeMap[16][16] = {{-1},{-1}};       
byte FFval = 255;
byte wallMap[33][33] =  { 
                          {'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'}, 
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
  
  mazeMap[x][y] = FFval;
  FFval--;
  y++;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main Loop Function
//Search Term: LOOPME
void loop(){
  rightForward(43);
  leftForward(37);
    
  while(encTickL < cellDistance-280){
    jankyPID();
  }
     
  rightBackward(80);
  leftBackward(90);
  while(encTickL < cellDistance){}
  
  rightBackward(0);
  leftBackward(0);
  checkSensors();
  
  while (wallRightFront){
       rightBackward(40);
       leftBackward(40);
       checkSensors();
  } 
  
  encTickL = 0;
  encTickR = 0;
  floodFill();
  //nextTurn = RIGHT;
  turn(nextTurn);
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
       leftForward(35);
       rightForward(55);
   }
   else{
      leftForward(37);
      rightForward(43); 
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
  if (recRead > 100){
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
  if (recRead > 665){
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
       uTurn(); break;
     default: break;    
  }
}

void checkSensors() {
  getIRLeft();
  getIRLeftFront();
  getIRRightFront();
  getIRRight(); 
  wallFront = wallLeftFront;
}

void turnLeft(){
  rightForward(100);
  leftBackward(100);
  while(encTickL < ticksForTurn-150){}
     
  rightBackward(90);
  leftForward(90);
  while(encTickL < ticksForTurn){}
  
  orientation--;
  encTickL = 0;
  encTickR = 0;
}

void turnRight(){
  rightBackward(100);
  leftForward(100);
  while(encTickL < ticksForTurn-100){}
     
  rightForward(90);
  leftBackward(90);
  while(encTickL < ticksForTurn){}
  
  orientation++;
  encTickL = 0;
  encTickR = 0;
}

void uTurn(){
 
  rightBackward(105);
  leftForward(100);
  while(encTickL < (uTicks - 400)){}
  rightForward(100);
  leftBackward(100);
  while(encTickL < uTicks){}
  
  encTickL = 0;
  encTickR = 0;
 orientation += 2; 
}

void floodFill(){

  if(FFval < mazeMap[x][y]) FFval = mazeMap[x][y];
  
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



