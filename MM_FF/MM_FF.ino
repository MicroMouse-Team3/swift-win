

#define MAXSIZE 255

int dx = 0, dy = 1;

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

  Point curr(dx,dy);
  int xPos = 0, yPos = 15;
      Stack s(MAXSIZE);


void setup() {

    Point initial(0,0);
  // put your setup code here, to run once:

    s.push(initial);
    
}




void loop() {
  
  if ( flood[xPos][yPos] > flood[xPos + (dx = -1)][yPos + (dy = 0)] ) {
    if ( 1 == 1 /*wall*/ )
  }
    ;
  else if ( flood[xPos][yPos] > flood[xPos][yPos + 1] /*AND no wall*/ )
    ;
  else if ( flood[xPos][yPos] > flood[xPos + dx][yPos + dy] )
    ;
  else
    ;
  
  
  if ( /*wallinFront*/1 == 1 )
    ;
    //turn
    //update floodfill
  else
    ;
    


  if ( curr.getX() == 0 && curr.getY() == 1 ) {
    flood[xPos + dx][yPos + dy];
  }
  while(1) {
    s.push(curr);
  }
  
}




/*
class Point {
  int x,y;
  
  public:
  Point(int x, int y) : x(x), y(y) {}
  int getX() { return x; }
  int getY() { return y; } 
  int dist() { return sqrt( (x * x) + (y * y) ); }
  
};
*/

/*
class Stack {
  int stack[MAXSIZE];
  int nelems;
  int top;
  
  public:
  Stack( int nelems ) : nelems(nelems) {}
  void push( int elem ) { stack[++top] = elem; }
  int pop() { return stack[--top]; }
};
*/
