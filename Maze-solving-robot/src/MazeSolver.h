#define MAZESOLVER_H

#include <Arduino.h>
#include "MotorPID.h"
#include "LineFollower.h" // reuse IR sensor pin definitions

// --- Hardware Pins ---
#define US_FRONT_TRIG 22
#define US_FRONT_ECHO 24
#define US_LEFT_TRIG 26
#define US_LEFT_ECHO 28
#define US_RIGHT_TRIG 37
#define US_RIGHT_ECHO 36

// IR pins and NUM_SENSORS come from LineFollower.h (IR1..IR8)

// --- Calibration Constants (TUNE THESE) ---
#define BASE_SPEED      70
#define TURN_SPEED      40
#define WALL_THRESHOLD  15.0f   // cm (If sensor reads < 15, it's a wall)
#define COUNTS_PER_90   230L    // Encoder counts for 90 deg turn
#define COUNTS_PER_CELL 550L    // Encoder counts for 18cm (1 cell) - MEASURE THIS!

// --- PD Wall Following constants ---
const float KP = 2.2; // Proportional gain (tune this)
const float KD = 0.0; // Derivative gain (tune this)
const float DESIRED_WALL_DISTANCE = 5.5; // Desired distance from wall in cm (tune this)


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
    bool isFinished(); // Returns true if target reached
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
