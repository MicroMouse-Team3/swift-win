#include <Motor.h>
#include <EncoderMM.h>
#include <LED.h>
#include <Sensor.h>
#include <NAV.h>

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


const int NUMSENSORS = 6;
volatile static int encTickL = 0, encTickR = 0;

int currentDirection = 4000;
int x = 0;
int y = 0;
int xprev = 0;
int yprev = 0;

//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
double previousError = 0.0;
int error = 0;
unsigned long curTime = 0;
unsigned long lastSamp = 0;
int delayTime = 0;
double previousPos = 0.0;
unsigned long  previousTime = 0;
unsigned long curt = 0; 
double stopError = 0.0;
long kp = 8L;
double kd = 0.126;
int curVel = 0;
int pwmPD = 0;
bool backWards = false; 

int lastTime = micros();
int lastError = 0;

//Encoder Information
volatile int state = LOW;

//Speeds of motors
int mapSpeed = 100;
int solveSpeed = 255;
 
 
//SENSOR Threshold Values
unsigned int minThresh = 15;
unsigned int maxThresh = 700;


unsigned long lastTickLeft = 0;
unsigned long lastTickRight = 0;

int ourOffset = 0;


boolean mapMode = true;

Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];

byte maze[16][16] =  { { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }, 
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
                      
byte dasMaze[33][33] =  { 
                          {'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1' }, 
                          {'1','0','1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'},                        
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
                          {'1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1','2','1'} ,
                          {'1','0','1','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','2','0','1'} ,
                          {'1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'} };



/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015. Hi
**/

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
  
  
  //initalize Motors
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , R_Mtr_B , R_CH_A , R_CH_B );
  
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
  
  //attach interrupts
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );    
  
  delay(5000);
  
  //LED Test
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
  
  //Determine Offset
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
  
   ourOffset = sensor[5]->getIR() - sensor[0]->getIR();
}
                                    
void loop(){

 byte turn;
  mtrL->setForward( mapSpeed );
  mtrR->setForward( mapSpeed );
  delay(500);
  //Gets us through the first half of the block just by going Straight.
  while(encTickL < 2000){
    curTime = micros(); 
    delayTime = curTime - lastSamp;
    lastSamp = curTime;
    
    pwmPD = P_error() + D_error();   
    
    if(pwmPD > 0){
      if(pwmPD > 100) pwmPD = 100;
      mtrL->setForward(pwmPD);
      mtrR->setForward(pwmPD);
    }
    else{
      pwmPD *= -1;
      if(pwmPD > 255) pwmPD = 255;
      mtrL->setBackward(pwmPD);
      mtrR->setBackward(pwmPD);
    }
  }
 
  mtrL->setBackward(0);
  mtrR->setBackward(0);
 
 delay(5000);
  encTickR = 0;
  encTickL = 0;
  
}

void speedControl(){
  
}

void turn(int thisTurn){
  
}

float P_error(){
  
 // float curPos = ((float)(encTickR + encTickL)/2.0);
  //float curVel = (curPos - previousPos)/((float)delayTime) ;
 // previousPos = curPos;
  previousError = curVel;
  curVel = 2000 - encTickL;
 
  return kp*curVel;
}
double D_error(){

  return kd*(curVel - previousError)/delayTime ;
}




int blockLength = 10;

void mapMaze() {
   maze[0][0] = -1;
   encTickL = encTickR = 0;             
}



/*
* PID Functions
*
*
**/
void PID(){
  int error = 0;
  int errorP = 0;
  int errorD = 0;
  static int oldErrorP = 0;
  int outputSpeed = 0;
  int integral =0, proportion=0, derivative=0;
  
  float KP = 1;
  float KD = 1;
  
  int left = sensor[0]->getIR();
  int right = sensor[5]->getIR();
  
  if (left > 600 && right > 600){
     errorP = right - left - ourOffset;
     errorD = errorP - oldErrorP;
  }
  else if(left > 600){
     errorP = 2*(820 - left);
     errorD = errorP - oldErrorP;  
  }
  else if (right > 600){
     errorP = 2*(right - 820);
     errorD = errorP - oldErrorP;  
  }
  else if (left < 600 && right < 600){
     errorP = 0; //(left encoder - right encoder)*3; 
    errorD = 0;  
}

error = (KP * errorP) + (KD * errorD);
oldErrorP = errorP;

mtrL->setForward( mapSpeed - error);
mtrR->setForward( mapSpeed + error);
}

void stopPID(){
  int error = 0;
  int errorP = 0;
  int errorD = 0;
  static int oldErrorP = 0;
  int outputSpeed = 0;
  int integral =0, proportion=0, derivative=0;
  
  float KP = 1;
  float KD = 1;
  
  int left = sensor[0]->getIR();
  int right = sensor[5]->getIR();
  
  if (left > 600 && right > 600){
     errorP = right - left - ourOffset;
     errorD = errorP - oldErrorP;
  }
  else if(left > 600){
     errorP = 2*(820 - left);
     errorD = errorP - oldErrorP;  
  }
  else if (right > 600){
     errorP = 2*(right - 820);
     errorD = errorP - oldErrorP;  
  }
  else if (left < 600 && right < 600){
     errorP = 0; //(left encoder - right encoder)*3; 
    errorD = 0;  
}

error = (KP * errorP) + (KD * errorD);
oldErrorP = errorP;

mtrL->setBackward( mapSpeed + error);
mtrR->setBackward( mapSpeed - error);
  
}
/********** ENCODER FUNCTIONS **********/
/*
* Encoder Tick Functions
*
*
**/
/****************MAPPPPPPING ********************/

byte NAV(){
  
  //0 = N, 1 = E, 2 = S, 3 = W  
  // 4= L, 5 = S, 6 = R, 7 = U
  
  //Determines wallLeft, wallRight, wallFront boolean values
  bool wallLeft = sensor[0]->getIR() > 600;
  bool wallFront = sensor[2]->getIR() > 200;
  bool wallRight = sensor[5]->getIR() > 600; 
  

  
  if(wallLeft){
     if(wallRight){
        if(wallFront){          
          return UTURN;         
        }
        else{
          return STRAIGHT;
        }
     }
    else{ //no wall right
       if(wallFront){
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
        if(wallFront){
          
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
       if(wallFront){
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
      
}

void determineOffset() {
  //  //Traps Setup until both front sensors are higher than 600.
//  digitalWrite(rightFrontLED, HIGH);
//  digitalWrite(leftFrontLED, LOW);
//  delay(500);
//  digitalWrite(leftFrontLED, LOW);
//  digitalWrite(rightFrontLED, HIGH);
//  delay(500);
//  digitalWrite(leftFrontLED, HIGH);
//  digitalWrite(rightFrontLED, LOW);
//  delay(500);
//  digitalWrite(leftFrontLED, LOW);
//  digitalWrite(rightFrontLED, HIGH);
//  delay(500);
//  digitalWrite(leftFrontLED, LOW);
//  digitalWrite(rightFrontLED, LOW);
  
  while ((sensor[2]->getIR() < 800) && (sensor[3]->getIR() < 800)){
    sensor[2]->getLED().setLOW();
    sensor[3]->getLED().setLOW();
    delay(128);
    sensor[2]->getLED().setHIGH();
    sensor[3]->getLED().setHIGH();
    delay(128);
  }
  
  encTickL = 0;
  encTickR = 0;

  sensor[2]->getLED().setHIGH();
  sensor[3]->getLED().setHIGH();    
  delay(500);
  sensor[2]->getLED().setLOW();
  sensor[3]->getLED().setLOW();    
  delay(500);
  sensor[2]->getLED().setHIGH();
  sensor[3]->getLED().setHIGH();    
  delay(500);
  sensor[2]->getLED().setLOW();
  sensor[3]->getLED().setLOW();
  
  ourOffset = sensor[5]->getIR() - sensor[0]->getIR();
}

 
int MAP(){
  return 0;
}

void initializeInterrupts() {
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
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

/*
void testSensors(){
  int testSense[NUMSENSORS];
  double sum = 0;  
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    test[i] = sensor[i]->getIR();
    sum += test[i];
  }
  delay(5000);
}
*/
void LEDsON() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getLED().setHIGH();
    delay(50);
  }
}
void LEDsOFF() {
  for ( byte i = 0 ; i < NUMSENSORS ; i++ ) {
    sensor[i]->getLED().setLOW();
    delay(50);
  }
}

void turnAround() {
  turnRight();
  turnRight(); 
}

void turnFullRight(){
   turnRight();
   turnRight(); 
   currentDirection++;
}

//90 degrees for turns
void turn90Right() { 
   encTickL = 0;
  encTickR = 0;
  mtrL->setForward(mapSpeed);
  mtrR->setBackward(mapSpeed);
  while(encTickR < 640){}
  mtrL->setBackward(mapSpeed);
  mtrR->setForward(mapSpeed);
  while(encTickR < 908){}
}

void turnRight(){
  encTickL = 0;
  encTickR = 0;
  mtrL->setForward(mapSpeed);
  mtrR->setBackward(mapSpeed);
  while(encTickR < 325){}
  mtrL->setBackward(mapSpeed);
  mtrR->setForward(mapSpeed);
  while(encTickR < 454){}
}

void turnLeft() {
  encTickL = 0;
  encTickR = 0;
  mtrL->setBackward(mapSpeed);
  mtrR->setForward(mapSpeed);
  while(encTickR < 300){}
  mtrL->setForward(mapSpeed);
  mtrR->setBackward(mapSpeed);
  while(encTickR < 454){}
  
  currentDirection--;
}
