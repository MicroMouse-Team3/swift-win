#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
using namespace std;

#define MAXSIZE 255
#define MAX 5

int row = 16, col = 16;

int flood[16][16] =  { { 14 , 13 , 12 , 11 , 10 ,  9 ,  8 ,  7 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14 }, 
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
                        
int dx = 0, dy = 1;
int xPos = 0, yPos = 15;
int val = 0;

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
      if ( nelems >= MAXSIZE ) {
          printf("PROBLEMS\n");
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
} q;

class CharArr {
    
public: 
};


/*void printPop() {
    printf("size = %d\n", s.size());
    val = s.pop();
    printf("%d\n", val);
}


void testStack(){
    for(int i = 0; i < MAX; i++)
        s.push(i);
    
    for(int i = 0; i < MAX; i++)
        printPop();
}*/

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


/*void testPointStack() {
    s.push( Point(30,30) );
    s.push( Point(31,31) );
    s.push( Point(32,32) );
    
    for(int i = 0; i < 3; i++){
        Point coord = s.pop();
        printf("Point %d: (%d, %d)\n", i, coord.getX(), coord.getY() );
    }
    
}
*/



    /*
  class Point {
  int x,y;
  
  public:
    Point(int x, int y) : x(x), y(y) {}
    int getX() { return x; }
    int getY() { return y; } 
    int dist() { return sqrt( (x * x) + (y * y) ); }
  };

class Stack {
  
  int nelems;
  int top;
  

  
  Point stack[MAXSIZE] = { Point(0,1), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0) };
  
  public:
  Stack( int nelems ) : nelems(nelems) {}
  void push( Point pt ) { stack[++top] = pt; }
  Point pop() { return stack[--top]; }
};
*/


 //     Stack s(MAXSIZE);

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
       //     printf("%d ", wallMap[i][j]);
        }
        if ( i != 0 && i % 2 == 0 )
            l++;
        printf("\n");
    }

    /*
    for (int i = 0; i < row; i++){
        for (int j = 0; j <= col ; j++){
            if( j == col)
                printf("%c", post);
            else {
                printf("%c", post);
                for(int k = 0; k < WALL_SIZE; k++) {
                    if(i == 0) { 
                            printf("%c", wallNBorder);
                    } else
                        printf("%c", wallN);
                }
            }
        }
        printf("\n");
        for ( int j = 0; j < col; j++){
            if ( j == 0)
                printf("%c  %2d  ", wallEBorder, flood[i][j]);
            else
                printf("%c  %2d  ", wallE, flood[i][j]);
        }
        printf("%c\n", wallEBorder);        
    }
    for ( int i = 0 ; i < row ; i++ ) {
        printf("%c", post);
        for(int i = 0; i < WALL_SIZE; i++) 
            printf("%c", wallNBorder);
    }
    printf("%c\n\n\n", post);
    */
}

int main() {        
  
      testPointStack();  
//    printMap();
    //testPointStack();
    
    
    

        
    //Point pt = new Point(0,0);
    //Point pt(0,0);
    //printf("%d\n", pt.getX());
    
    
/*  Point ptArr[3] = { Point(0,0), Point(1,1), Point(2,2) };
    
    for(int i = 0; i < 3; i++){
        printf("Point %d: (%d, %d)\n", i, ptArr[i].getX(), ptArr[i].getY() );
    }*/
    
//  Stack s(5);
    
    //testStack();
    

    
    
    
    //cout << "Point: (" << pt.getX() << ", " << pt.getY() << ")\n";
    //put your setup code here, to run once:
    //s.push(initial);
    





//void loop() {
  
  //if ( flood[xPos][yPos] > flood[xPos + (dx = -1)][yPos + (dy = 0)] ) {
   // if ( 1 == 1 /*wall*/ )
 // }
    ;
 // else if ( flood[xPos][yPos] > flood[xPos][yPos + 1] /*AND no wall*/ )
    ;
 // else if ( flood[xPos][yPos] > flood[xPos + dx][yPos + dy] )
    ;
  //else
    ;
  
  
  //if ( /*wallinFront*/1 == 1 )
    ;
    //turn
    //update floodfill
  //else
   // ;
    


 // if ( curr.getX() == 0 && curr.getY() == 1 ) {
 //   flood[xPos + dx][yPos + dy];
 // }
 // while(1) {
    //s.push(curr);
//  }
  
}


/*
class Stack {
  int stack[MAXSIZE];
  int nelems;
  int top;
  
  public:
  Stack( int nelems ) : nelems(nelems), top(0) {}
  void push( int elem ) { stack[++top] = elem; }
  int pop() { return stack[top--]; }
  int size() { return top; }
} s(MAX);
*/

/*
class Stack {
  Point stack[MAXSIZE] = { Point(0,1), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0),
                      Point(0,0), Point(0,0), Point(0,0), Point(0,0), Point(0,0) }; 
                      
  int nelems;
  int top;


  public:
  Stack( int nelems ) : nelems(nelems), top(0) {
      this->nelems = nelems;
      this->top = 0;
      for(int i = 0; i < MAXSIZE; i++)
          stack[i] = Point(0,0); 
  }
  
  void push( Point pt ) { stack[++top] = Point(pt.getX(), pt.getY() ); }
  Point pop() { return stack[top--]; }
  int size() { return top; }
} s(MAX);
*/

/*

void printMap(){
    
    bool isWall = false;
    
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
    
    for (int i = 0; i < row; i++){
        for (int j = 0; j <= col ; j++){
            if( j == col)
                printf("%c", post);
            else {
                printf("%c", post);
                for(int k = 0; k < WALL_SIZE; k++) {
                    if(i == 0) { 
                            printf("%c", wallNBorder);
                    } else
                        printf("%c", wallN);
                }
            }
        }
        printf("\n");
        for ( int j = 0; j < col; j++){
            if ( j == 0)
                printf("%c  %2d  ", wallEBorder, flood[i][j]);
            else
                printf("%c  %2d  ", wallE, flood[i][j]);
        }
        printf("%c\n", wallEBorder);        
    }
    for ( int i = 0 ; i < row ; i++ ) {
        printf("%c", post);
        for(int i = 0; i < WALL_SIZE; i++) 
            printf("%c", wallNBorder);
    }
    printf("%c\n\n\n", post);
}
*/
