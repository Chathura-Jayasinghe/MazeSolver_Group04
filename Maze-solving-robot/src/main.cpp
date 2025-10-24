#include <Arduino.h>
#include "MotorPID.h"
#include "MazeSolver.h"
#include "LineFollower.h"

// =============================================
// PIN DEFINITIONS
// =============================================

// Motor 1 (Left) pins
#define MOTOR1_IN1 8
#define MOTOR1_IN2 9
#define ENCODER1_A 3
#define ENCODER1_B 7

// Motor 2 (Right) pins
#define MOTOR2_IN1 5
#define MOTOR2_IN2 6
#define ENCODER2_A 2
#define ENCODER2_B 4

// IR Sensor pins
#define IR1 43
#define IR2 44
#define IR3 45
#define IR4 46
#define IR5 47
#define IR6 48
#define IR7 49
#define IR8 50

// =============================================
// CONSTANTS
// =============================================
const int irPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
const int numSensors = 8;
const int MAX_MAZE_CYCLES = 25;  // Maximum cycles in maze solving mode

// =============================================
// ROBOT STATE CONSTANTS
// =============================================
#define STATE_MAZE_SOLVING 0
#define STATE_TRANSITIONING_TO_LINE_FOLLOW 1
#define STATE_LINE_FOLLOWING 2

// =============================================
// GLOBAL VARIABLES
// =============================================
int robotState;
bool currentRunMode = false;      // false = Maze Solving, true = Line Following
int mazeCycleCount = 1;          // Counter for maze solving cycles
bool lineDetectionReady = false; // Flag to indicate ready for line following

// =============================================
// MOTOR AND CONTROLLER OBJECTS
// =============================================
MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);
LineFollower lineFollower(leftMotor, rightMotor);


// =============================================
// INITIALIZATION FUNCTIONS
// =============================================
void initializeHardware() {
    leftMotor.begin();
    rightMotor.begin();
}

void initializeControllers() {
    mazeSolver.begin();
    mazeSolver.setIRPins(irPins, numSensors);  // Share IR pins with MazeSolver
    lineFollower.begin();
}

void initializeRobotState() {
    robotState = STATE_MAZE_SOLVING;
    currentRunMode = false;      // Start with maze solving
    mazeCycleCount = 1;
    lineDetectionReady = false;
}

void setup() {
    Serial.begin(9600);
    
    // Initialize all components
    initializeHardware();
    initializeControllers();
    initializeRobotState();

    // Startup messages
    Serial.println("Robot Initialized");
    Serial.println("Starting in 2 seconds...");
    Serial.println("Mode: Line Following (will switch to Maze Solving when all white detected)");
    delay(2000);
}

// =============================================
// MAIN CONTROL FUNCTIONS
// =============================================
void handleMazeSolving() {
    if (mazeCycleCount == MAX_MAZE_CYCLES) {
        lineDetectionReady = true;
    }

    mazeSolver.readSensors();
    mazeSolver.forwardForMs(80, 15);
    // Serial.println("Maze Solving Mode Active - Moving Forward");
    mazeSolver.moveForwardWithWallFollowing();
    // Serial.println("Switching to Line Following -- move forward for a while");
    // delay(1000);
    mazeCycleCount++;
    // Serial.println(mazeCycleCount);
    // Serial.println("Starting Line Following");
}


void handleLineFollowing() {
    lineFollower.update();
    Serial.println("Line Following Mode Active");
}

void handleMazeDetection() {
    bool allWhite = mazeSolver.update();
    if (allWhite) {
        currentRunMode = true;  // Switch to maze solving mode
        Serial.println("--------------------Switching to Line Following - Mazesolver Complete-------------------------");
    }
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
    if (currentRunMode && !(mazeCycleCount > MAX_MAZE_CYCLES)) {
        handleMazeSolving();
    } 
    else if (lineDetectionReady) {
        handleLineFollowing();
    } 
    else {
        handleMazeDetection();
    }
}

