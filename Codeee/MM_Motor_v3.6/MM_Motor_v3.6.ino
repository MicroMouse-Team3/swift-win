/* MicroMouse [SwirlBot] Motor code

[insert more information about motors here... when not lazy]

/*


NOTE:  I CHANGED PINS FROM YOURS!!  I WAS SO CONFUSED,
          I THOUGHT THEY ALL HAD TO BE ON PWM pins x]
          Ooops, well it still works, so you might have to change
          pins to fit your design, ill talk to you more about what I am suppose
          to connect... cause I dun understand it :c

NOTE:  I pretty much ONLY have L-side setup (nothing for Right)
          I changed ALL the pin names to reflect that all of the
          variables are associated with LEFT side
          
          This will make changing pins and copy-pasting EASY for RIGHT side
          
          Feel free to change anything :]
          

          I saved a backup copy

NOTE:  "Right" side code has NOT BEEN TESTED, proceed with caution!!! Double check it!
          Also, NONE of the pins have been assigned and everything is commented OUT

*/

#include "initialize.h"

/********** START OF setup() **********/
void setup() {
  //Enable serial
  Serial.begin(9600);
  Serial.print("START\n");  
  
  //sensor[NUMSENSORS] = { (LeftSensor,leftLED) , new Sensor(FrontSensor,frontLED) , new Sensor(RightSensor,rightLED) };
  sensor[0] = new Sensor(LeftSensor,leftLED);
  sensor[1] = new Sensor(FrontSensor,frontLED);
  sensor[2] = new Sensor(RightSensor,rightLED);
  
  mtrL = new Motor( L_Enable , L_Mtr_A , L_Mtr_B , L_CH_A , L_CH_B );  
  mtrR = new Motor( R_Enable , R_Mtr_A , L_Mtr_B , R_CH_A , R_CH_B );
  //Set LEFT motor to move FORWARD

  delay(5000);
  setBothMtrsForward();
}
/********** END OF setup() **********/


/********** START OF loop() **********/
void loop() {  
  readBothEnc();
  readSensors();
  printSensorReadings();
  sensorsToLEDs();

  if(isWallAllAround()) {
   mtrL->setSpeed(0);
   mtrR->setSpeed(0);
  }
  else { 
    mtrL->setSpeed(50);
    mtrR->setSpeed(50);
  }
//  hopeEyeNeverHitWall();
 
//  squareMoveTest();
  
  //test1();

}

/********** END OF loop() **********/

bool isWallLeft() {
  if ( sensor[0]->getSensorReading() < minThresh )
    return true;
    
  return false;
}

bool isWallFront() {    
  if ( sensor[1]->getSensorReading() < minThresh )
    return true;
  
  return false;
}

bool isWallRight() {
  if ( sensor[2]->getSensorReading() < minThresh )
    return true;
  
  return false;
}

bool isWallFrontAndRight() {
  if ( isWallFront() && isWallRight() )
    return true;
  
  return false;
}

bool isWallFrontAndLeft() {
  if ( isWallLeft() && isWallFront() )
    return true;
  
  return false;
}

bool isWallAllAround() {
  if ( isWallLeft() && isWallFront() && isWallRight() )
    return true;
  
  return false;
}


void hopeEyeNeverHitWall() {
  //Always moving forward
  
  //wall in front ONLY - naturally turn RIGHT
  /*********************************************
  
                  @@@@@
                  @   @      -->
                  @   @
   */
  if ( isWallFront() ) {
    turnRight();
  }
  //wall in front and LEFT - turn right
  /*********************************************
             *
             *    @@@@@
             *    @   @      -->
             *    @   @
             *
             *
   */
   if ( isWallFrontAndRight() ) {
     turnRight();
   }
  
  //wall in front and RIGHT - turn left
    /*********************************************
                            *
                  @@@@@     *
        <--       @   @     *
                  @   @     *
                            *
                            *
   */  
   if ( isWallFrontAndLeft() ) {
     turnLeft();
   }
  //wall in front and LEFT and RIGHT - uTurn , naturally turning RIGHT
      /*********************************************
            *               *
            *     @@@@@     *
            *     @   @     *
            *     @   @     *
            *               *
            *     |--|      *
            *     |  |      *
            *        V      *
   */
   if ( isWallAllAround() ) {
     turnAround();
   }
}

void squareMoveTest() {
  
  for ( byte i = 0 ; i < 4 ; i++ ) {
    
    speed = 50;
    setBothMtrsSpd(speed);
  
    //Move forward for 3 seconds
    delay(3000);
  
    //turnRight  
    turnRight();
  }
}

void turnRight() {
  speed = 0;
  setBothMtrsSpd(speed);
  delay(1000);
  setMtrsRightTurn();
  speed = 50;
  setBothMtrsSpd(speed);  
  delay(10);
  speed = 0;
  setBothMtrsSpd(speed); 
    
  delay(1000);
  setBothMtrsForward();
  delay(1000);
}

void turnLeft() {
  speed = 0;
  setBothMtrsSpd(speed);
  delay(1000);
  setMtrsLeftTurn();
  speed = 50;
  setBothMtrsSpd(speed);  
  delay(10);
  speed = 0;
  setBothMtrsSpd(speed); 
    
  delay(1000);
  setBothMtrsForward();
  delay(1000);
}

void turnAround() {
  speed = 0;
  setBothMtrsSpd(speed);
  delay(1000);
  setMtrsRightTurn();
  speed = 50;
  setBothMtrsSpd(speed);  
  delay(20);
  speed = 0;
  setBothMtrsSpd(speed); 
    
  delay(1000);
  setBothMtrsForward();
  delay(1000);
}

void test1() {
    //ALL three to disable motor
  if ( (sensor[0]->getSensorReading() < minThresh) && (sensor[1]->getSensorReading() < minThresh) && (sensor[2]->getSensorReading() < minThresh) )
    setLMtrSpd(speed = 0);
  else// if ( sensorReading[0] > maxThresh || sensorReading[1] > maxThresh || sensorReading[2] > maxThresh )
    setLMtrSpd(speed = 50);
}

/********** START OF TEST FUNCTIONS **********/

/*
void squareMoveTest() {
  delay (1000);
  speed = 50;
  
  L_Enc_Pos = R_Enc_Pos = 0;
  setBothMtrsForward();
  analogWrite(L_Enable, speed);
  //analogWrite(R_Enable, speed);
  for ( L_Enc_Pos = 0, R_Enc_Pos = 0 ; L_Enc_Pos < leftTicks && R_Enc_Pos < rightTicks ; )
     readBothEnc();
     
  speed = 0;
  analogWrite(L_Enable, speed);
  //analogWrite(R_Enable, speed);   
  
  
  delay(500);
  
  speed = 0;
  setMtrsRightTurn();
  analogWrite(L_Enable, speed);
  //analogWrite(R_Enable, speed);  
  
  delay(500);
  
}
void turnTest() {
  
  // left,front,right sensor are ON
    //Turn 180 degrees (LEFT)
  if ( LED[0] == HIGH && LED[1] == HIGH && LED[2] == HIGH )
    uTurn();
  
  // left,front are ON ; right is OFF
    //turn 90 degrees (RIGHT)
  if ( LED[0] == HIGH && LED[1] == HIGH && LED[2] == LOW )
    rightTurn();
  
  // left is OFF ; front,right are ON
    //turn 90 degrees (LEFT)
  if ( LED[0] == LOW && LED[1] == HIGH && LED[2] == HIGH ) {
    leftTurn();
  }
}

void uTurn() {
  //Will naturally turn LEFT
  
  
  
  //These are based off calibrations for a 90 degree turn for each LEFT and RIGHT motor
    //Left Enc going (-)
    //Right Enc going (+)
  leftTicks = -50;
  rightTicks = 50;
  
  setMtrsLeftTurn();
  speed = 40;
  
  for ( L_Enc_Pos = 0, R_Enc_Pos = 0 ; L_Enc_Pos > leftTicks && R_Enc_Pos < rightTicks ; L_Enc_Pos--, R_EncPos++ ) {
     readBothEnc();
  }  
}

void leftTurn() {
  
  
  //These are based off calibrations for a 45 degree turn for each LEFT and RIGHT motor
    //Left Enc going (-)
    //Right Enc going (+)
  leftTicks = -25;
  rightTicks = 25;

  setMtrsLeftTurn();
  speed = 40;
  for ( L_Enc_Pos = 0, R_Enc_Pos = 0 ; L_Enc_Pos > leftTicks && R_Enc_Pos < rightTicks ; L_Enc_Pos--, R_EncPos++ ) {
     readBothEnc();
  }  
  
}
void rightTurn() {  
  //These are based off calibrations for a 45 degree turn for each LEFT and RIGHT motor
    //Left Enc going (-)
    //Right Enc going (+)
  leftTicks = 25;
  rightTicks = -25;

  setMtrsRightTurn();
  speed = 40;
  for ( L_Enc_Pos = 0, R_Enc_Pos = 0 ; L_Enc_Pos < leftTicks && R_Enc_Pos > rightTicks ; L_Enc_Pos++, R_EncPos-- ) {
     readBothEnc();
  }
}



void test() {
  readBothEnc();
  
  
  //Current test functions being tested
  
  //reverseLMtr();
  changeSpd();
}





void changeSpd() {
  
  switch( L_Enc_Pos ) {
    
    case(50): 
      speed = 200;
      break;
      
      
    case(150):
      speed = 50;
      break;
    case(225):
      speed = 0;
      break;
      
  }
}


void reverseLMtr() {
  
  if ( L_Enc_Pos >= 50 ) {
    moveLBackward();
  }
  else if ( L_Enc_Pos <= -50 ) {
    moveLForward();
  }
}
*/

/********* END OF TEST FUNCTIONS **********/
