#define MAZESOLVER_H

#include <Arduino.h>
#include "MotorPID.h"
#include "LineFollower.h" 

// --- Hardware Pins ---
#define US_FRONT_TRIG 22
#define US_FRONT_ECHO 24
#define US_LEFT_TRIG 26
#define US_LEFT_ECHO 28
#define US_RIGHT_TRIG 37
#define US_RIGHT_ECHO 36

// --- Calibration Constants (TUNE THESE) ---
#define BASE_SPEED      70
#define TURN_SPEED      40
#define WALL_THRESHOLD  15.0f  
<<<<<<< HEAD
#define COUNTS_PER_90   240L    //230L thibuneeee
=======
#define COUNTS_PER_90   240L
>>>>>>> c68fb1f4a44033cd06149dc3ba9ceb391ec4d4e3
#define COUNTS_PER_CELL 525L   

// --- PD Wall Following constants ---
const float KP = 2.2;
const float KD = 0.0; 
const float DESIRED_WALL_DISTANCE = 5.5;


// --- Maze Constants ---
#define MAZE_SIZE 8
#define TARGET_X  1
#define TARGET_Y  4

// --- Direction Enums (Clockwise) ---
enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

// --- Wall Bitmasks ---
#define WALL_NORTH (1 << NORTH)
#define WALL_EAST  (1 << EAST)
#define WALL_SOUTH (1 << SOUTH)
#define WALL_WEST  (1 << WEST)

class MazeSolver {
public:
    MazeSolver(MotorPID& left, MotorPID& right);
    void begin();
    
    // Main loop function
    void runStep(); 
    bool isFinished();
    bool isTargetDetectedIR();
    void reset();
    void computeShortestPath();
    void followShortestPathStep();

private:
    MotorPID& leftMotor;
    MotorPID& rightMotor;

    // Robot State
    int currX, currY;
    Direction currDir;
    bool MET_FRONT_WALL;
    
    // The Map
    byte walls[MAZE_SIZE][MAZE_SIZE];
    int  dist[MAZE_SIZE][MAZE_SIZE];

    // Shortest path storage
    Direction path[MAZE_SIZE * MAZE_SIZE];
    int pathLen;
    int pathIndex;

    // Helper Functions
    float readSensor(int trig, int echo);
    void  updateWalls();
    void  floodFill();
    Direction getBestDirection();
    Direction getBestDirectionAt(int x, int y);
    
    // Movement Primitives
    void turnTo(Direction targetDir);
    void moveOneCell();
    void turnLeft();
    void turnRight();
    void turnAround();
    void stopMotors();
    
    // Encoders
    long encLeft();
    long encRight();
    void encZero();
};
