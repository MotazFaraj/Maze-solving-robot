#include <PID_v1_bc.h>
#include <NewPing.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>


/* Constants */



#define MAZE_SIZE 8

#define START_X 0
#define START_Y 0

#define INFINITE 9999 

int current_x = 0, current_y = 0;



int goal_position[2] = {3, 3};



enum DIRECTION {NORTH, SOUTH, EAST , WEST };

// the first state the robot will go to north
enum DIRECTION current_direction = NORTH;

// We will change one of them to -1, after detect the first direction is left(west) or right(east)
// if the first direction is left(west) then east_state will be equal to -1
// elif the first direction is right(east) then the west_state will be equal to -1
int east_state = 1;
int west_state = 1;

bool direction_weight_done = false;




enum VISIT {NOT_VISITED, VISITED};

enum BLOCK { NOT_BLOCKED, BLOCKED};


struct Cell {

    int pos_x;
    int pos_y;



    int heuristic_cost; // Manhattan Distance (from current cell to the goal cell)
    int actual_cost; // Distance (from current cell to the start cell)

    int parent_x;
    int parent_y;

    enum BLOCK  blocked;

};


struct Point {
    int x;
    int y;
};


struct Cell maze[MAZE_SIZE][MAZE_SIZE];

enum VISIT visited[MAZE_SIZE][MAZE_SIZE];

bool finished_scan = false;
bool finished_returned = false;
bool finished_solution = false;

bool visited_allowed = false; // for backward to the parent that is visited




/*Constants */

// Define constants
#define ENA 11
#define ENB 10
#define in1 13
#define in2 12
#define in3 9
#define in4 8
#define IR_RIGHT_PIN 7
#define IR_LEFT_PIN 6
#define ENCODER_RIGHT_PIN 3
#define ENCODER_LEFT_PIN 2
#define ECHO_FRONT_PIN A5
#define TRIGGER_FRONT_PIN A4
#define ECHO_RIGHT_PIN A3
#define TRIGGER_RIGHT_PIN A2
#define ECHO_LEFT_PIN A1
#define TRIGGER_LEFT_PIN A0

const int maxDistance = 400;
const int minDistance = 12;
const int encoderResolution = 200;  // Number of encoder ticks per revolution
double pidKp = 0.5;
double pidKi = 0.0;
double pidKd = 0.0;
////////////////////////////////////
double pidKp1 = 0.5;
double pidKi1 = 0.0;
double pidKd1 = 0.0;
////////////////////
const double LeftSetpoint = 300, RightSetpoint = 290;
// Define variables
double LeftInput = 0, RightInput = 0;
double LeftOutput = 0, RightOutput = 0;
double LeftSpeed = 0, RightSpeed = 0;
int LeftEncoderValue = 0, RightEncoderValue = 0;
int LeftMotorPulses = 0, RightMotorPulses = 0;
unsigned long lastTime = 0;
bool leftWall, rightWall, frontWall;
float frontSensor, oldFrontSensor;
float leftSensor;
float rightSensor;

// Create PID objects
PID LeftPID(&LeftInput, &LeftOutput, &LeftSetpoint, pidKp1, pidKi1, pidKd1, DIRECT);
PID RightPID(&RightInput, &RightOutput, &RightSetpoint, pidKp, pidKi, pidKd, DIRECT);

// Setup ultrasonic sensors
NewPing sonarFront(TRIGGER_FRONT_PIN, ECHO_FRONT_PIN, maxDistance);
NewPing sonarRight(TRIGGER_RIGHT_PIN, ECHO_RIGHT_PIN, maxDistance);
NewPing sonarLeft(TRIGGER_LEFT_PIN, ECHO_LEFT_PIN, maxDistance);


/* Functions */



void changeDirecton(enum DIRECTION newDirection){
    if(!direction_weight_done){
        if(newDirection == EAST){
            west_state = -1;
            direction_weight_done = true;
        }
        if(newDirection == WEST){
            east_state = -1;
            direction_weight_done = true;
        }
    }
    current_direction = newDirection;
}

// when make forward
// and we have different directions
void update_position(int* x, int* y){

    if(current_direction == NORTH){
        *y += 1;
    }else if (current_direction == SOUTH)
    {
        *y += -1;
    }else if (current_direction == WEST)
    {
        *x += west_state;
    }else if (current_direction == EAST)
    {
        *x += east_state;
    }
    
}

bool check_reached_goal(int* x, int* y){

  
    if(*x == goal_position[0] && *y == goal_position[1])
        return true;
        
  

    return false;

}

bool check_reached_start(int* x, int* y){

  
    if(*x == START_X && *y == START_Y)
        return true;
        
  

    return false;

}

bool check_all_visited(){


    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            if(visited[i][j] == NOT_VISITED)
                return false;
        }
        
    }
    
    return true;
}


int manhattanDistance(int x, int y){
    return abs(x - goal_position[0]) + abs(y - goal_position[1]);
}

int h(int x, int y){
    return manhattanDistance(x, y);
}


void initialize_MAZE(){

    for (int i = 0; i < MAZE_SIZE; i++)
    {
         for (int j = 0; j < MAZE_SIZE; j++)
        {
            visited[i][j] == NOT_VISITED;
        }
    }

  

    maze[START_X][START_Y].parent_x = -1;
    maze[START_X][START_Y].parent_y = -1;

    for (int i = 0; i < MAZE_SIZE; i++)
    {
         for (int j = 0; j < MAZE_SIZE; j++)
        {
            maze[i][j].pos_x = i;
            maze[i][j].pos_y = j;

           

            maze[i][j].heuristic_cost = h(i, j);
            maze[i][j].actual_cost = INFINITE;

            maze[i][j].blocked = NOT_BLOCKED;

            
        }
    }

    maze[START_X][START_Y].actual_cost = 0;
    

} 


int getCellCost(int x, int y){
    if(x == -1 || y == -1)
        return INFINITE * 2;
    return  maze[x][y].heuristic_cost + maze[x][y].actual_cost;
}

bool checkValidCell(int x, int y){

    if(x >= MAZE_SIZE || y >= MAZE_SIZE || x < 0 || y < 0)
        return false;

    
    // if(maze[x][y].blocked == BLOCKED)
    //     return false;

    return true;
}

void update_neighbour(int neighbour_x, int neighbour_y ){

    if(!checkValidCell(neighbour_x, neighbour_y))
        return;

    int heuristic_cost = h(neighbour_x, neighbour_y);
    int actual_cost = maze[current_x][current_y].actual_cost + 1;

    // fprintf(stderr, "c:x: %d, y: %d \n",  current_x, current_y);
    // fflush(stderr);
    

    if(getCellCost(neighbour_x, neighbour_y) > (heuristic_cost + actual_cost)){
        maze[neighbour_x][neighbour_y].actual_cost = actual_cost;
        maze[neighbour_x][neighbour_y].heuristic_cost = heuristic_cost;
        maze[neighbour_x][neighbour_y].parent_x = current_x;
        maze[neighbour_x][neighbour_y].parent_y = current_y;
    }

    // fprintf(stderr, "h1:x: %d, y: %d \n",  current_x, current_y);
    // fflush(stderr);

    // fprintf(stderr, "h2:x: %d, y: %d \n",  maze[neighbour_x][neighbour_y].actual_cost, maze[neighbour_x][neighbour_y].heuristic_cost);
    // fflush(stderr);

}


bool checkFrontWall(){
    return frontWall;
    //return API_wallFront();
}

bool checkLeftWall(){
    return leftWall;
    //return API_wallLeft();
}

bool checkRightWall(){
    return rightWall;
    //return API_wallRight();
}

void moveForward(){
    Forward();
    //runMotors();
    //API_moveForward();
}

void turnLeft(){
    Left();
    //API_turnLeft();
}

void turnRight(){
    Right();
    //API_turnRight();
}

void turnBack(){
    Back();

    //API_turnRight();
    //API_turnRight();

}
void update_neighbours(){

   
   
     if(current_direction == NORTH){
        if(!checkFrontWall() ){
            update_neighbour(current_x, current_y + 1);
        }
        if(!checkLeftWall()){
            update_neighbour((current_x + west_state), current_y);
        }
        if(!checkRightWall() ){
            update_neighbour((current_x + east_state), current_y);
        }
    }else if (current_direction == SOUTH)
    {
        if(!checkFrontWall()){
            update_neighbour(current_x, current_y - 1);
        }
        if(!checkLeftWall()){
            update_neighbour((current_x + east_state), current_y);
        }
        if(!checkRightWall()){
            update_neighbour((current_x + west_state), current_y);
        }
    }else if (current_direction == WEST)
    {
        if(!checkFrontWall()){
            update_neighbour((current_x + west_state), current_y);
        }
        if(!checkLeftWall()){
            update_neighbour(current_x, current_y - 1);
        }
        if(!checkRightWall()){

            update_neighbour(current_x, current_y + 1);
        }
    }else if (current_direction == EAST)
    {
        if(!checkFrontWall()){
            update_neighbour((current_x + east_state), current_y);
        }
        if(!checkLeftWall()){
            update_neighbour(current_x, current_y + 1);
        }
        if(!checkRightWall()){
            update_neighbour(current_x, current_y - 1);
        }
    }

}


void moveToCell(int new_pos_x, int new_pos_y){

    if(current_direction == NORTH){
        // y++
        if(new_pos_x == current_x && new_pos_y > current_y){
            moveForward();
            update_position(&current_x, &current_y);
        }
        // y--
        else if(new_pos_x == current_x && new_pos_y < current_y){
            turnBack();
            
            changeDirecton(SOUTH);

            moveForward();
            update_position(&current_x, &current_y);
        }
        // x++
        else if(new_pos_x > current_x && new_pos_y == current_y){

            if(east_state == 1){
                turnRight();
                
                changeDirecton(EAST);      
            }else{
                turnLeft();
                
                changeDirecton(WEST);  

            }
            

            moveForward();
            update_position(&current_x, &current_y);
        }
        // x--
        else if(new_pos_x < current_x && new_pos_y == current_y){
            

            if(east_state == 1){
                turnLeft();
                changeDirecton(WEST);      
            }else{
                turnRight(); 
                changeDirecton(EAST);    
            }

            moveForward();
            update_position(&current_x, &current_y);
        }
    }else if (current_direction == SOUTH)
    {
        if(new_pos_x == current_x && new_pos_y > current_y){
            turnBack();
            
            changeDirecton(NORTH);   

            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x == current_x && new_pos_y < current_y){
           
            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x > current_x && new_pos_y == current_y){
           

            if(east_state == 1){
                turnLeft();
                
                changeDirecton(EAST);       
            }else{
                turnRight();
                
                changeDirecton(WEST); 

            }

            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x < current_x && new_pos_y == current_y){
            

            if(east_state == 1){
                turnRight();
                
                changeDirecton(WEST); 
      
            }else{
                turnLeft();
                
                changeDirecton(EAST);
            }

            moveForward();
            update_position(&current_x, &current_y);
        }
    }else if (current_direction == WEST)
    {
        if(new_pos_x == current_x && new_pos_y > current_y){
            turnRight();
            
            changeDirecton(NORTH);

            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x == current_x && new_pos_y < current_y){
            turnLeft();
            
            changeDirecton(SOUTH);
           
            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x > current_x && new_pos_y == current_y){

            if(east_state == 1){
                turnBack();
                
                changeDirecton(EAST);
            }
            
            moveForward();
            update_position(&current_x, &current_y);
        }
        else if(new_pos_x < current_x && new_pos_y == current_y){
            

            if(east_state == 1){
                // nothing
            } else {
                turnBack();
                
                changeDirecton(EAST);
            }

            moveForward();
            update_position(&current_x, &current_y);
        }
    }else if (current_direction == EAST)
    {
        // y++
       if(new_pos_x == current_x && new_pos_y > current_y){
            turnLeft();
            
            changeDirecton(NORTH);

            moveForward();
            update_position(&current_x, &current_y);
        }
        // y--
        else if(new_pos_x == current_x && new_pos_y < current_y){
            turnRight();
            
            changeDirecton(SOUTH);
           
            moveForward();
            update_position(&current_x, &current_y);
        }
        // x++
        else if(new_pos_x > current_x && new_pos_y == current_y){

            

             if(east_state == 1){
                // nothing
            } else {
                turnBack();
                
                changeDirecton(WEST);

            }
            
            moveForward();
            update_position(&current_x, &current_y);
        }
        // x--
        else if(new_pos_x < current_x && new_pos_y == current_y){

            if(east_state == 1){
                turnBack();
                
                changeDirecton(WEST);
            } 
            
            moveForward();
            update_position(&current_x, &current_y);
        }
    }

}


void moveToParent(){
    // fprintf(stderr, "o:%d  p:%d\n", maze[current_x][current_y].parent_x, maze[current_x][current_y].parent_y);
    // fflush(stderr); 
    moveToCell(maze[current_x][current_y].parent_x, maze[current_x][current_y].parent_y);
}

struct Point getTheLowestCellCost(struct Point point1, struct Point point2, struct Point point3, bool solution){
    if ( getCellCost(point1.x, point1.y) <= getCellCost(point2.x, point2.y) && getCellCost(point1.x, point1.y) <= getCellCost(point3.x, point3.y)){
       
        if(solution || (visited[point1.x][point1.y] == NOT_VISITED))
            return point1;
    }
    else if (getCellCost(point2.x, point2.y) <= getCellCost(point1.x, point1.y) && getCellCost(point2.x, point2.y) <= getCellCost(point3.x, point3.y))
    {
        if(solution || (visited[point2.x][point2.y] == NOT_VISITED))
            return point2;
    }

    else if(solution || (visited[point3.x][point3.y] == NOT_VISITED)){
        return point3;
    }

    
        
       
    // when all the points is visited then go back to the parent
    struct Point backward;
    backward.x = -1;
    backward.y = -1;  

    return backward;
    
}

struct Point getTheLowestNeighbourCostCell(bool solution){

    struct Point point1, point2, point3;
    point1.x = -1;
    point1.y = -1;
    point2.x = -1;
    point2.y = -1;
    point3.x = -1;
    point3.y = -1;

    

    if(current_direction == NORTH){
        
        if(!checkFrontWall() && checkValidCell(current_x, current_y + 1)){
            
            point1.x = current_x;
            point1.y = current_y + 1;
        }
        if(!checkLeftWall() && checkValidCell((current_x + west_state), current_y)){
            
            point2.x = (current_x + west_state);
            point2.y = current_y;
           
        }
        if(!checkRightWall()  && checkValidCell((current_x + east_state), current_y)){
            
            point3.x = (current_x + east_state);
            point3.y = current_y;
            
        }
    }else if (current_direction == SOUTH)
    {
        if(!checkFrontWall() && checkValidCell(current_x, current_y - 1)){
            point1.x = current_x;
            point1.y = current_y - 1;
        }
        if(!checkLeftWall()  && checkValidCell((current_x + east_state), current_y)){
            point2.x = (current_x + east_state);
            point2.y = current_y;
        }
        if(!checkRightWall() && checkValidCell((current_x + west_state), current_y)){
            point3.x = (current_x + west_state);
            point3.y = current_y;
        }
    }else if (current_direction == WEST)
    {
        if(!checkFrontWall() && checkValidCell((current_x + west_state), current_y)){
            point1.x = (current_x + west_state);
            point1.y = current_y;
        }
        if(!checkLeftWall() && checkValidCell(current_x, current_y - 1)){
            point2.x = current_x;
            point2.y = current_y - 1;
        }
        if(!checkRightWall() && checkValidCell(current_x, current_y + 1)){
            point3.x = current_x;
            point3.y = current_y + 1;
           
        }
    }else if (current_direction == EAST)
    {
        if(!checkFrontWall() && checkValidCell((current_x + east_state), current_y)){
            point1.x = (current_x + east_state);
            point1.y = current_y;
        }
        if(!checkLeftWall() && checkValidCell(current_x, current_y + 1)){
            point2.x = current_x;
            point2.y = current_y + 1;
        }
        if(!checkRightWall() && checkValidCell(current_x, current_y - 1)){
            point3.x = current_x;
            point3.y = current_y - 1;
        }
    }

   

    return getTheLowestCellCost(point1, point2, point3, solution);

}

bool checkNotVisitedNeighbour(int x, int y){
    if(checkValidCell(x, y)){
        return visited[x][y] == NOT_VISITED ? true : false;
    }
    return false;
    
}
struct Point getNeighbourNotVisited(){

    struct Point neighbour;
    neighbour.x = -1;
    neighbour.y = -1;

     if(current_direction == NORTH){
        if(!checkFrontWall()  && checkValidCell(current_x, current_y + 1) && checkNotVisitedNeighbour(current_x, current_y + 1)){
             neighbour.x = current_x;
             neighbour.y = current_y + 1;
        }
        else if(!checkLeftWall()  && checkValidCell((current_x + west_state), current_y) && checkNotVisitedNeighbour((current_x + west_state), current_y)){
            neighbour.x = (current_x + west_state);
             neighbour.y = current_y;
        }
        else if(!checkRightWall()  && checkValidCell((current_x + east_state), current_y) && checkNotVisitedNeighbour((current_x + east_state), current_y)){
            neighbour.x = (current_x + east_state);
            neighbour.y = current_y;
        }
    }else if (current_direction == SOUTH)
    {
        if(!checkFrontWall() && checkValidCell(current_x, current_y - 1) && checkNotVisitedNeighbour(current_x, current_y - 1)){
             neighbour.x = current_x;
             neighbour.y = current_y - 1;
        }
        else if(!checkLeftWall() && checkValidCell((current_x + east_state), current_y) && checkNotVisitedNeighbour((current_x + east_state), current_y)){
            neighbour.x = (current_x + east_state);
             neighbour.y = current_y;
        }
        else if(!checkRightWall() && checkValidCell((current_x + west_state), current_y) && checkNotVisitedNeighbour((current_x + west_state), current_y)){
             neighbour.x = (current_x + west_state);
             neighbour.y = current_y;
        }
    }else if (current_direction == WEST)
    {
        if(!checkFrontWall() && checkValidCell((current_x + west_state), current_y) && checkNotVisitedNeighbour((current_x + west_state), current_y)){
             neighbour.x = (current_x + west_state);
             neighbour.y = current_y;
        }
        else if(!checkLeftWall() && checkValidCell(current_x, current_y - 1) && checkNotVisitedNeighbour(current_x, current_y - 1)){
             neighbour.x = current_x;
             neighbour.y = current_y - 1;
        }
        else if(!checkRightWall() && checkValidCell(current_x, current_y + 1) && checkNotVisitedNeighbour(current_x, current_y + 1)){
             neighbour.x = current_x;
             neighbour.y = current_y + 1;
        }
    }else if (current_direction == EAST)
    {
        if(!checkFrontWall() && checkValidCell((current_x + east_state), current_y) && checkNotVisitedNeighbour((current_x + east_state), current_y)){
            neighbour.x = (current_x + east_state);
            neighbour.y = current_y;
        }
        else if(!checkLeftWall() && checkValidCell(current_x, current_y + 1) && checkNotVisitedNeighbour(current_x, current_y + 1)){
            neighbour.x = current_x;
            neighbour.y = current_y + 1;
        }
        else if(!checkRightWall() && checkValidCell(current_x, current_y - 1) && checkNotVisitedNeighbour(current_x, current_y - 1)){
            neighbour.x = current_x;
            neighbour.y = current_y - 1;
        }
    }

    return neighbour;

}


/* Functions */

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), readEncoderRight, RISING);
  //Initialize IR pins
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  // Initialize encoder pins
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  // Initialize PWM pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  //Initialize H-bridge direction
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Set PID tuning parameters
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetSampleTime(10);
  LeftPID.SetOutputLimits(0, 255);
  RightPID.SetMode(AUTOMATIC);
  RightPID.SetSampleTime(10);
  RightPID.SetOutputLimits(0, 255);
}

void loop() {
 

    initialize_MAZE();
    
    while (!finished_scan)
    {
        if(visited[current_x][current_y] == NOT_VISITED)
            visited_allowed = false;

        visited[current_x][current_y] = VISITED;

    
         walls(); // check the walls for the current cell


        struct Point neighbourCell;


        // update the cost for cell neighbours that no has wall between current cell and them
        // if the new cost is lower than old then update it and put the new parent to the neighbour cell to the current cell
        if (!visited_allowed){
            update_neighbours();
            neighbourCell = getTheLowestNeighbourCostCell(false); // get point not visited

        

            // fprintf(stderr, "x: %d, y: %d \n", neighbourCell.x, neighbourCell.y);
            // fflush(stderr);

            // fprintf(stderr, "m  :x: %d, y: %d \n",  current_x, current_y);
            // fflush(stderr);
    

            if(neighbourCell.x == -1 || neighbourCell.y == -1) {
                
                visited_allowed = true;
                moveToParent();

            } else {
                // update the cell that i want to go to, put the parent of it current cell
                maze[neighbourCell.x, neighbourCell.y]->parent_x = current_x;
                maze[neighbourCell.x, neighbourCell.y]->parent_y = current_y;


                

                // it will move to new cell and will update the new position
                // and change the direction if required
                moveToCell(neighbourCell.x, neighbourCell.y);

            }


        } else {
            neighbourCell = getNeighbourNotVisited();

            
            if(neighbourCell.x == -1 || neighbourCell.y == -1) {
                visited_allowed = true;
                // blocked this cell 
                maze[current_x][current_y].blocked = BLOCKED;
                moveToParent();

            }else {
                visited_allowed = false;
                moveToCell(neighbourCell.x, neighbourCell.y);
            }
        }
        
        

        if(check_reached_goal(&current_x, &current_y))
            finished_scan = true;
    }


    //delay(5); // wait 5 second

    // return to the start point 

    while (!finished_returned)
    {
        

        
        moveToParent();


        if(check_reached_start(&current_x, &current_y))
            finished_returned = true;


    }



    //delay(5); // wait 5 second



  
    // change the direction from south to north when we reached the start point
   
    turnBack();
    changeDirecton(NORTH);
    

    // return to the goal point by check the cost from the start point

    while (!finished_solution)
    {
        

        
        struct Point neighbourCell;
        neighbourCell = getTheLowestNeighbourCostCell(true); // get point visited not care


        moveToCell(neighbourCell.x, neighbourCell.y);

        if(check_reached_goal(&current_x, &current_y))
            finished_solution = true;


    }


    



}

// Function to read encoder value
int readEncoderLeft() {
  LeftEncoderValue++;
  LeftMotorPulses++;
}

int readEncoderRight() {
  RightEncoderValue++;
  RightMotorPulses++;
}

void Forward() {
  // ReadLeft();
  // ReadRight();
  double prevPos = 0.5 * (leftSensor - rightSensor);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  pid(prevPos);
  Stop(1);
  delay(40);
  Stop(0);
  delay(500);
}

void Left() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  pid2(90);
  Stop(1);
  Stop(0);
  delay(500);
}

void Right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  pid2(90);
  Stop(1);
  Stop(0);
  delay(500);
}

void Back() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  pid2(180);
  Stop(1);
  Stop(0);
  delay(500);
}

void Stop(int op) {
  if (op) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(ENA, 0);
  } else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
    analogWrite(ENB, 0);
  }
}

void pid(double prevPos) {
  int pulsesLeft = 216, pulsesRight = 235, first = 1;
  double currPos;
  LeftMotorPulses = 0;
  RightMotorPulses = 0;
  double start = millis();
  while (millis() < start + 10000) {
    if (millis() - lastTime >= 1000) {
      LeftSpeed = LeftEncoderValue * 60 / 189;
      RightSpeed = RightEncoderValue * 60 / 206;
      LeftEncoderValue = 0;
      RightEncoderValue = 0;
      lastTime = millis();
    }
    LeftInput = LeftSpeed;
    RightInput = RightSpeed;
    // Compute PID outputs
    LeftPID.Compute();
    RightPID.Compute();
    // Read ultra sonic sensors
    ReadFront();
    ReadLeft();
    ReadRight();
    currPos = 0.5 * (leftSensor - rightSensor);
    // Set motor speeds
    if (frontSensor <= minDistance)
      break;
    if (prevPos - currPos > 1 && !first) {
      analogWrite(ENB, 0.9 * LeftOutput);
      analogWrite(ENA, 1.1 * RightOutput);
    } else if (prevPos - currPos < -1 && !first) {
      analogWrite(ENB, 1.1 * LeftOutput);
      analogWrite(ENA, 0.9 * RightOutput);
    } else {
      analogWrite(ENB, LeftOutput);
      if(first){
        delay(100);
        first--;
      }
      analogWrite(ENA, RightOutput);
    }
  }
}

void pid2(int deg) {
  int pulsesLeft, pulsesRight;
  if (deg == 90) {
    pulsesLeft = 89, pulsesRight = 97;
  } else {
    pulsesLeft = 178, pulsesRight = 194;
  }
  LeftMotorPulses = 0;
  RightMotorPulses = 0;
  while (1) {
    if (millis() - lastTime >= 10000) {
      LeftSpeed = LeftEncoderValue * 60 / 189;
      RightSpeed = RightEncoderValue * 60 / 200;
      LeftEncoderValue = 0;
      RightEncoderValue = 0;
      lastTime = millis();
    }
    LeftInput = LeftSpeed;
    RightInput = RightSpeed;
    // Compute PID outputs
    LeftPID.Compute();
    RightPID.Compute();
    // Set motor speeds
    analogWrite(ENB, LeftOutput);
    analogWrite(ENA, RightOutput);
    Serial.println(LeftMotorPulses);
    if(LeftMotorPulses >= 200)
      break;
  }
}

void walls() {
  if (digitalRead(IR_LEFT_PIN) == LOW) {
    leftWall = true;
  } else {
    leftWall = false;
  }

  if (digitalRead(IR_RIGHT_PIN) == LOW) {
    rightWall = true;
  } else {
    rightWall = false;
  }

  ReadFront();
  if (frontSensor < minDistance) {
    frontWall = true;
  } else {
    frontWall = false;
  }
}

void ReadFront() {
  do {
    frontSensor = sonarFront.ping() / 29.41 / 2.0;
  } while (frontSensor <= 0);
}

void ReadRight() {
  do {
    rightSensor = sonarRight.ping() / 29.41 / 2.0;
  } while (rightSensor <= 0);
}

void ReadLeft() {
  do {
    leftSensor = sonarLeft.ping() / 29.41 / 2.0;
  } while (leftSensor <= 0);
}
