#include <EncoderMM.h>
#include <LED.h>
#include <Motor.h>
#include <Sensor.h>

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

int currentDirection = 4000;
int x = 0;
int y = 0;

unsigned long lastTickLeft = 0;
unsigned long lastTickRight = 0;

const int NUMSENSORS = 6;
volatile static int encTickL = 0, encTickR = 0;
int ourOffset = 0;

Motor * mtrL;
Motor * mtrR;
Sensor * sensor[NUMSENSORS];


//Global Boolean Values
const bool debugOn = "TRUE";
const bool solved = "FALSE";

//Global Values
const int distancePerMove = 30;
long int delayLastError = 0;
int previous_error = 0;
unsigned long curt = 0; 

//Encoder Information
volatile int state = LOW;

//Speeds of motors
int mapSpeed = 128;
int solveSpeed = 255;
 
 
//SENSOR Threshold Values
unsigned int minThresh = 15;
unsigned int maxThresh = 700;



/*
*  MAIN for MicroMouse
*  This is the only piece of code that will be uploaded to the Micromouse
*  Team 3 - Winter 2015. Hi
**/

int lastTime = millis();
int lastError = 0;

void setup(){
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
  sensor[2] = new Sensor( leftFrontEmitIR , leftFrontRecIR , leftFrontLED );
  //Right Front
  sensor[3] = new Sensor( rightFrontEmitIR , rightFrontRecIR , rightFrontLED );
  //Right Diag
  sensor[4] = new Sensor( rightDiagEmitIR , rightDiagRecIR , rightDiagLED );
  //Right Right
  sensor[5] = new Sensor( rightEmitIR , rightRecIR , rightLED ); 
    
  attachInterrupt( L_CH_A , incEncoderL , RISING );
  attachInterrupt( R_CH_A , incEncoderR , RISING );
  
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


boolean mapMode = true;
byte maze[16][16] = { { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }, 
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
  encTickR = 0;
  encTickL = 0;
  byte turn;
  mtrL->setForward( mapSpeed );
  mtrR->setForward( mapSpeed );
  
  //Gets us through the first half of the block just by going Straight.
  while(encTickL < 1000){
    PID();
  }
  
  //After half way, determines which way we should turn ahead.
  turn = NAV();
  
  //Contiunes straight until it is time to stop.
  while(encTickL < 1810  ){
    PID();
  }
  mtrL->setBackward(mapSpeed);
  mtrR->setBackward(mapSpeed);
  
  while(encTickL < 2000){
    stopPID();
  }
  
  delay(500);
  
  
  //turnRight();
  
  mtrL->setBackward(0);
  mtrR->setBackward(0);
  
 delay(5000);
  
  
}

int current_error(){
  int curPos = (encTickR + encTickL)/2;
  int error = curPos - 1334;
  previous_error = error;
  return error;
}
int P_error(){
  delayLastError = millis();
  return current_error() * 1;
}
int D_error(){
  return (current_error() - previous_error)/(millis() - delayLastError); 
}
void testSensors(){
  int test1 = sensor[0]->getIR();
  int test2 = sensor[1]->getIR();
  int test3 = sensor[2]->getIR();
  int test4 = sensor[3]->getIR();
  int test5 = sensor[4]->getIR();
  int test6 = sensor[5]->getIR();
  test6 = test5 + test4 + test3 + test2 + test1;
  delay(5000);
}
void LEDsON() {
  digitalWrite(rightLED, HIGH); delay(50);
        digitalWrite(rightFrontLED,HIGH); delay(50);
        digitalWrite(rightDiagLED,HIGH); delay(50);
        digitalWrite(leftLED, HIGH); delay(50);
        digitalWrite(leftFrontLED, HIGH); delay(50);
        digitalWrite(leftDiagLED, HIGH); delay(50);
}
void LEDsOFF() {
         digitalWrite(rightLED, LOW); delay(50);
        digitalWrite(rightFrontLED,LOW); delay(50);
        digitalWrite(rightDiagLED,LOW); delay(50);
        digitalWrite(leftLED, LOW); delay(50);
        digitalWrite(leftFrontLED, LOW); delay(50);
        digitalWrite(leftDiagLED, LOW); delay(50);
}

int blockLength = 10;

void mapMaze() {
   maze[0][0] = -1;
   encTickL = encTickR = 0;             
}

void turnAround() {
  turnRight();
  turnRight(); 
}

//90 degrees for turns
void turnRight() { 
  encTickL = 0;
  encTickR = 0;
  mtrL->setForward(mapSpeed);
  mtrR->setBackward(mapSpeed);
  while(encTickR < 300){}
  mtrL->setBackward(mapSpeed);
  mtrR->setForward(mapSpeed);
  while(encTickR < 454){}
  
  currentDirection++;
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



/****************MAPPPPPPING ********************/

byte NAV(){
  
  //0 = N, 1 = E, 2 = S, 3 = W

  
  // 4= L, 5 = S, 6 = R, 7 = U
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
