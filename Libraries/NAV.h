
#ifndef NAV_h
#define NAV_h

#include "Arduino.h"

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

  
#endif
