#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
using namespace std;

#define MAXSIZE 255
#define MAX 5

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

bool northDir = true, eastDir = true, southDir = true, westDir = true;


int mapX = 31, mapY = 1;
int row = 16, col = 16;

int dx = 0, dy = 1;
int posX = 15, posY = 0;
int val = 0;

int flood[16][16] =  { { 14 , 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14 }, 
                       { 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 }, 
                       { 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 }, 
                       { 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 }, 
                       { 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 }, 
                       {  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 }, 
                       {  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 }, 
                       {  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  0 ,  0 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 }, 
                       {  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  0 ,  0 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 }, 
                       {  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  1 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 }, 
                       {  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  2 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 }, 
                       { 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  3 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 }, 
                       { 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  4 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 }, 
                       { 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  5 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 }, 
                       { 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  6 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 }, 
                       { 14 , 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14 } }; 
                       
int wallMap[33][33] =  { {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1}, 
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},  
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3}, 
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3}, 
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3}, 
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}, 
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}, 
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}, 
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
                         {3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1} };

//East Walls = 3 North Walls = 2
//Only change 0's to 2's or 3's. 
int mapEx[33][33] =  {   {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1}, 
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3},  
                         {1,0,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,0,1,2,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,3}, 
                         {1,2,1,2,1,2,1,0,1,2,1,2,1,2,1,2,1,0,1,2,1,2,1,2,1,0,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,3,8,0,8,0,8,0,8,0,8,3,8,0,8,0,8,0,8,0,8,3,8,3,8,0,8,3},
                         {1,0,1,0,1,0,1,2,1,2,1,2,1,0,1,2,1,0,1,2,1,2,1,2,1,0,1,2,1,0,1,0,1},
                         {3,8,3,8,3,8,3,8,0,8,0,8,0,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3},
                         {1,0,1,0,1,0,1,2,1,0,1,2,1,2,1,0,1,2,1,2,1,2,1,2,1,0,1,2,1,2,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3}, 
                         {1,2,1,2,1,0,1,2,1,2,1,2,1,2,1,0,1,0,1,0,1,0,1,2,1,0,1,2,1,0,1,0,1},
                         {3,8,0,8,0,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,3,8,3}, 
                         {1,0,1,2,1,2,1,0,1,2,1,2,1,2,1,2,1,2,1,0,1,2,1,2,1,2,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,3,8,0,8,0,8,0,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,3},
                         {1,2,1,2,1,0,1,2,1,2,1,2,1,2,1,2,1,2,1,0,1,0,1,2,1,2,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,0,8,3,8,0,8,0,8,0,8,0,8,3,8,3,8,3},
                         {1,0,1,2,1,0,1,2,1,2,1,2,1,2,1,0,1,0,1,0,1,2,1,2,1,0,1,2,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,0,8,3,8,0,8,0,8,0,8,0,8,3,8,3,8,3},
                         {1,2,1,2,1,2,1,2,1,0,1,2,1,2,1,0,1,2,1,0,1,2,1,2,1,2,1,0,1,0,1,0,1},
                         {3,8,0,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,3,8,3,8,3},
                         {1,0,1,2,1,2,1,0,1,2,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,2,1,0,1,0,1}, 
                         {3,8,0,8,0,8,3,8,3,8,0,8,3,8,3,8,0,8,3,8,0,8,0,8,0,8,0,8,3,8,3,8,3},
                         {1,2,1,2,1,0,1,0,1,0,1,2,1,0,1,2,1,2,1,0,1,0,1,0,1,2,1,0,1,0,1,0,1}, 
                         {3,8,0,8,0,8,3,8,0,8,3,8,0,8,3,8,0,8,3,8,0,8,0,8,0,8,0,8,0,8,3,8,3},
                         {1,2,1,0,1,2,1,0,1,2,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,2,1,2,1,0,1},
                         {3,8,0,8,0,8,0,8,3,8,0,8,3,8,3,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,2,1,0,1,2,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,2,1,0,1},
                         {3,8,0,8,0,8,3,8,0,8,3,8,3,8,3,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,2,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,2,1,0,1}, 
                         {3,8,3,8,3,8,0,8,3,8,3,8,3,8,3,8,3,8,3,8,0,8,0,8,0,8,0,8,0,8,0,8,3},
                         {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,2,1,0,1,2,1},
                         {3,8,3,8,0,8,3,8,3,8,0,8,3,8,0,8,0,8,0,8,3,8,0,8,3,8,0,8,0,8,0,8,3},
                         {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1} };
                         
                       


                        


class Point {
    int x,y;
  
  public:
  Point(int x, int y) : x(x), y(y) {}
  int getX() { return x; }
  int getY() { return y; } 
  int dist() { return sqrt( (x * x) + (y * y) ); }
};



class Queue {
  Point queue[MAXSIZE] = { Point(0,1), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0) }; 
                      
  int head, tail, nelems;


  public:
  Queue() : nelems(0), head(0), tail(0) {
      for(int i = 0; i < MAXSIZE; i++)
          queue[i] = Point(i,i); 
  }
  
  bool enQ( Point pt ) { 
      if ( nelems > MAXSIZE ) {
          printf("PROBLEMS ");
          printf("%d", nelems);
          printf(" . I expected less then 255.\n");
          return false;
      }
          
      queue[tail] = Point(pt.getX(), pt.getY() );
      
      if ( tail == MAXSIZE - 1 )
          tail = 0;
      else
          tail++;
          
      nelems++;
      return true;
  }
  Point deQ() { 
      
      Point tmp = queue[head];
      
      if ( head == MAXSIZE - 1 )
          head = 0;
      else
          head++;
      
      nelems--;
      return tmp;
  }
  int size() { return nelems; }
  bool isEmpty() {
      if (nelems == 0 )
          return true;
      return false;
  }
} q;

void testPointStack() {
    q.enQ( Point(30,30) );
    q.enQ( Point(31,31) );
    q.enQ( Point(32,32) );
    q.deQ();
    for(int i = 0; i < 2; i++){
        Point coord = q.deQ();
        printf("Point %d: (%d, %d)\n", i, coord.getX(), coord.getY() );
    }
    
}


void printMap(){
    
    bool isWall = true;
    
    const int WALL_SIZE = 6;
    char post = '+';
    char wallE = '|';
    char wallN = '-';//[WALL_SIZE] = { '-', '-', '-', '-', '-', '-' };
    char wallNBorder = '-';
    
    char wallEBorder = '|';
    
    if(!isWall){
        wallE = ' ';
        wallN = ' ';
        //for (int i = 0; i < WALL_SIZE; i++)
          //  wallN[i] = ' ';
    }
    
    for ( int i = 0, l = 0 ; i < row * 2 + 1 ; i++ ) {
        for ( int j = 0, m = 0 ; j < col * 2 + 1 ; j++ ) {
            if ( wallMap[i][j] == 8 )
                printf("  %2d  ", flood[l][m]);
            else if ( wallMap[i][j] == 1 )
                printf("%c", post);
            else if ( wallMap[i][j] == 2 ) {
                for ( int i = 0 ; i < WALL_SIZE ; i++ )
                    printf("%c", wallN);
            } else if ( wallMap[i][j] == 3 )
                printf("%c", wallE);
            else if ( wallMap[i][j] == 0 ) {
                if ( i % 2 == 0 )
                    printf("      ");
                else
                    printf(" ");               
            }
            if ( j != 0 && j % 2 == 0 )
                m++;     
        }
        if ( i != 0 && i % 2 == 0 )
            l++;
        printf("\n");
    }
}

void printMapEx(){
    
    bool isWall = true;
    
    const int WALL_SIZE = 6;
    char post = '+';
    char wallE = '|';
    char wallN = '-';//[WALL_SIZE] = { '-', '-', '-', '-', '-', '-' };
    char wallNBorder = '-';
    
    char wallEBorder = '|';
    
    if(!isWall){
        wallE = ' ';
        wallN = ' ';
    }
    
    for ( int i = 0, l = 0 ; i < row * 2 + 1 ; i++ ) {
        for ( int j = 0, m = 0 ; j < col * 2 + 1 ; j++ ) {
            if ( mapEx[i][j] == 8 )
                printf("  %2d  ", flood[l][m]);
            else if ( mapEx[i][j] == 1 )
                printf("%c", post);
            else if ( mapEx[i][j] == 2 ) {
                for ( int i = 0 ; i < WALL_SIZE ; i++ )
                    printf("%c", wallN);
            } else if ( mapEx[i][j] == 3 )
                printf("%c", wallE);
            else if ( mapEx[i][j] == 0 ) {
                if ( i % 2 == 0 )
                    printf("      ");
                else
                    printf(" ");               
            }
            if ( j != 0 && j % 2 == 0 )
                m++;
       //     printf("%d ", wallMap[i][j]);
        }
        if ( i != 0 && i % 2 == 0 )
            l++;
        printf("\n");
    }
}


void updateFloodMap() {
    
}

void checkOpenCells() {

    northDir = true, eastDir = true, southDir = true, westDir = true;
    int mapX = 1, mapY = 31;
    Point mapPt(mapX,mapY);
    if ( wallMap[mapX][mapY + 1] != 0 )
        northDir = false;
    if ( wallMap[mapX + 1][mapY] != 0 )
        eastDir = false;
    if ( wallMap[mapX][mapY - 1] != 0 )
        southDir = false;
    if ( wallMap[mapX - 1][mapY] != 0 )
        westDir = false;
}

Point moveCell(Point currPos) {
    Point pos = currPos;
 
    // int min = flood[pos.getX()][pos.getY() + 1];
    
    if ( eastDir == true ) {
        printf("Going EAST\n");
        mapY += 2;
        return Point(pos.getX(),pos.getY()+1);
    }
    if ( northDir == true ) {
        printf("Going NORTH\n");
        mapX -= 2;
        return Point(pos.getX() - 1,pos.getY());
    }
     if ( westDir == true){
         printf("Going WEST\n");
         mapY -= 2;
        return Point(pos.getX(),pos.getY()-1);
     }
     if ( southDir == true ) {
        printf("Going SOUTH\n");
        mapX += 2;
        return Point(pos.getX() + 1,pos.getY());
    }
    
    
    /*
    if ( northDir == true ) {
        if ( eastDir == true ) {
            if ( southDir == true ) {
                if ( eastDir == true ) {
                    //Check FF in all dir
                   min = flood[pos.getX()][pos.getY() + 1];
                   if ( flood[pos.getX() - 1][pos.getY()] < min )
                       min = flood[pos.getX() - 1][pos.getY()];
                   if ( flood[pos.getX()][pos.getY() - 1] < min )
                       min = flood[pos.getX()][pos.getY() - 1];
                   if ( flood[pos.getX() + 1][pos.getY()] < min )
                       min = flood[pos.getX() + 1][pos.getY()];
                }            
                //Check FF: N, E, S
                
                min = flood[pos.getX()][pos.getY() + 1];
                if ( flood[pos.getX() - 1][pos.getY()] < min )
                   min = flood[pos.getX() - 1][pos.getY()];
                if ( flood[pos.getX()][pos.getY() - 1] < min )
                   min = flood[pos.getX()][pos.getY() - 1];
            }
            //Check FF: N, E

            min = flood[pos.getX()][pos.getY() + 1];
            if ( flood[pos.getX() - 1][pos.getY()] < min )
               min = flood[pos.getX() - 1][pos.getY()];
        }
        //U turn
    }
    */
        
}

int main() { 
    //printf("I'm in loop\n");
    printMap();       
    printMapEx();
    int dir = NORTH;
    Point currPos(posX,posY);
    //Point nextPos(posX + dx, posY - dy);
    Point coord(0,0);
    int currDist;
    int md;

    int counter = 0;
    Point adjCoord(posX,posY);
    northDir = true, eastDir = true, southDir = true, westDir = true;
    
    printf("%d", wallMap[mapX][mapY - 1]);
    printf("%d", wallMap[mapX][mapY + 1]);
    printf("%d", wallMap[mapX+1][mapY]);
    printf("%d", wallMap[mapX-1][mapY]);
    
    for ( int i = 0 ; i < row * 2 + 1 ; i++ ) {
        for ( int j = 0 ; j < col * 2 + 1 ; j++ ) {
            wallMap[i][j] = mapEx[i][j];
        }
    }

        q.enQ(currPos);
    
    do {
        counter++;
        
        printf("%d", wallMap[mapX][mapY - 1]);
        printf("%d", wallMap[mapX][mapY + 1]);
        printf("%d", wallMap[mapX+1][mapY]);
        printf("%d", wallMap[mapX-1][mapY]);
        printf("I'm in the do-while!: My number is:  ");
        printf("%d\n", counter);
        printf("Coord(%d,%d)\n", mapX, mapY);
   
        northDir = true, eastDir = true, southDir = true, westDir = true;
  
        
        coord = q.deQ();
        
        currDist = flood[coord.getX()][coord.getY()];
//        printf("%d\n", md);        

        
        Point mapPt(mapX,mapY);
        
        
        //There is a wall north
        if ( wallMap[mapX-1][mapY] != 0 ) {
            printf("There is a north wall\n");
            northDir = false;
            adjCoord = Point(posX-1,posY);
            md = flood[adjCoord.getX()][adjCoord.getY()];
            if ( md != currDist - 1 ) {
                flood[coord.getX()][coord.getY()] = md + 1;
                //Add neighbors to Q
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() - 1) );
                q.enQ( Point(adjCoord.getX() + 1,adjCoord.getY()) );
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() + 1) );
                q.enQ( Point(adjCoord.getX() - 1,adjCoord.getY()) );
            }
        }        
        //if wall to east
        if ( wallMap[mapX][mapY+1] != 0 ) {
            printf("There is a east wall\n");
            eastDir = false;
            adjCoord = Point(posX,posY+1);
            md = flood[adjCoord.getX()][adjCoord.getY()];
            if ( md != currDist - 1 ) {
                flood[coord.getX()][coord.getY()] = md + 1;
                //Add neighbors to Q
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() - 1) );
                q.enQ( Point(adjCoord.getX() + 1,adjCoord.getY()) );
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() + 1) );
                q.enQ( Point(adjCoord.getX() - 1,adjCoord.getY()) );
            }            
        }
        //south
        if ( wallMap[mapX + 1][mapY] != 0 ) {
            printf("There is a south wall\n");
            southDir = false;
            adjCoord = Point(posX + 1,posY);
            md = flood[adjCoord.getX()][adjCoord.getY()];
            if ( md != currDist - 1 ) {
                flood[coord.getX()][coord.getY()] = md + 1;
                //Add neighbors to Q
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() - 1) );
                q.enQ( Point(adjCoord.getX() + 1,adjCoord.getY()) );
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() + 1) );
                q.enQ( Point(adjCoord.getX() - 1,adjCoord.getY()) );
            }            
        }
        //west
        if ( wallMap[mapX][mapY - 1] != 0 ) {
            printf("There is a west wall\n");
            westDir = false;
            adjCoord = Point(posX,posY - 1);
            md = flood[adjCoord.getX()][adjCoord.getY()];
            if ( md != currDist - 1 ) {
                flood[coord.getX()][coord.getY()] = md + 1;
                //Add neighbors to Q
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() - 1) );
                q.enQ( Point(adjCoord.getX() + 1,adjCoord.getY()) );
                q.enQ( Point(adjCoord.getX(),adjCoord.getY() + 1) );
                q.enQ( Point(adjCoord.getX() - 1,adjCoord.getY()) );
            }
        }
        
        


                
             
        //checkOpenCells();
        currPos = moveCell(currPos);
                
        q.enQ(currPos);
    //    printMap(**mapEx);        
        printMap();
    } while ( !q.isEmpty() );
     
     printMap();   
    printf("Aww fuck~!\n");
    
 //   updateFloodMap();
 
  
}