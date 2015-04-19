//TABLE OF CONTENTS (SEARCH FOR THESE TERMS)
////////////////////////////
//Defintions -> DEFINEME
//Initilizations -> INITME
//Setup -> SETME
//Loop -> LOOPME
//EncoderTick -> TICKME
//Turn -> TURNME
//Error Functions -> ERRME
//Get Sensor Functions -> SENME
//--------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Definitions
//Search Term: DEFINEME
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Initializations
//INITME
//For Mapping
int currentDirection = 4000;
int x = 0;
int y = 0;
int xprev = 0;
int yprev = 0;
const bool solved = "FALSE";

boolean mapMode = true;
bool wallLeft = false;
bool wallLeftDiag = false;
bool wallLeftFront = false;
bool wallRightFront = false;
bool wallRightDiag = false;
bool wallRight = false; 

//For Movement
const int distancePerMove = 30;
int mapSpeed = 100;
int solveSpeed = 255; 

//For PID

// PID gains
double Tkp = 8L;
double Tkd = 6300L;
double Wkp = 1;
double Wkd =1;

// Error Tracking
double ERROROLD = 0;
double error = 0;
double CURRENTTIME = 0;
double LASTTIME =0;
int DIRECTION = 0;

// Setpoints
int CELLDISTANCE = 2200;
int SETPOINT = CELLDISTANCE;
int SETVAL = 0;
int wallOFFSET = 123;

// Sensor offsets
int wallLEFTDIST = 0;
int wallRIGHTDIST = 0;

//PWM vars

int PWMRATE = 0;


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
  ourOffset = getIRLeft() - getIRRight();
  LASTTIME = micros();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Loop Functions
//Search Term: LOOPME
void loop(){
  
  while(encTickL < CELLDISTANCE/2 + SETVAL){
    PWMRATE = speedControl();
    PID(PWMRATE);
  }
  DIRECTION = NAV();
  if(DIRECTION == STRAIGHT;
  if(wallLeftFront && wallRightFront)
  SETPOINT -= WALLOFFSET;
  
  
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
void turnRight(){
  encTickL = 0;
  encTickR = 0;
  leftForward(mapSpeed);
  rightBackward(mapSpeed);
  while(encTickR < 325){}
  leftBackward(mapSpeed);
  rightForward(mapSpeed);
  while(encTickR < 454){}
  currentDirection++;
}

void turnLeft(){
  encTickL = 0;
  encTickR = 0;
  leftBackward(mapSpeed);
  rightForward(mapSpeed);
  while(encTickR < 300){}
  leftForward(mapSpeed);
  rightBackward(mapSpeed);
  while(encTickR < 454){}
  currentDirection--;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Error Functions
//ERRME
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//NAV Function
//NAVME
byte NAV(){
  
  //0 = N, 1 = E, 2 = S, 3 = W  
  // 4= L, 5 = S, 6 = R, 7 = U
  
  //Determines wallLeft, wallRight, wallFront boolean values

  wallLeft = getIRLeft() > 600;
  wallLeftFront = getIRLeftFront() > 200;
  wallRight = getIRRight() > 600; 


  
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
      
}










