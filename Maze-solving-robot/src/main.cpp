#include <Arduino.h>
#include "MotorPID.h"
#include "MazeSolver.h"

// --- Hardware Definitions ---
#define MOTOR1_IN1 8
#define MOTOR1_IN2 9
#define ENCODER1_A 3
#define ENCODER1_B 7

#define MOTOR2_IN1 5
#define MOTOR2_IN2 6
#define ENCODER2_A 2
#define ENCODER2_B 4

// --- Individual Switch Pins ---
#define SWITCH_EXPLORE 31      // Switch 1: ON = Explore maze mode
#define SWITCH_SHORTEST_PATH 33 // Switch 2: ON = Shortest path mode  
#define SWITCH_RESET 35       // Switch 3: ON = Reset memory

// --- Robot States ---
enum RobotState {
    IDLE = 0,           // All switches OFF - Idle/Standby
    EXPLORE = 1,        // Switch 1 ON - Search and store maze
    SHORTEST_PATH = 2,  // Switch 2 ON - Follow shortest path
    RESET_MODE = 3      // Switch 3 ON - Reset memory
};

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);

// Global state variables
RobotState currentState = IDLE;
RobotState previousState = IDLE;
bool isRunning = false;
bool isFinished = false;
bool pathFollowingMode = false;

// Function prototypes
RobotState readSwitchState();
void printCurrentState(RobotState state);
void handleStateChange();

void setup() {
    Serial.begin(9600);
    
    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();

    // Initialize switch pins
    pinMode(SWITCH_EXPLORE, INPUT_PULLUP);
    pinMode(SWITCH_SHORTEST_PATH, INPUT_PULLUP);
    pinMode(SWITCH_RESET, INPUT_PULLUP);

    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║     FLOOD FILL MAZE SOLVER v3.0       ║");
    Serial.println("║        3-Switch Controlled             ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Switch Controls:");
    Serial.println("  Switch 1 ON  - EXPLORE MAZE MODE");
    Serial.println("  Switch 2 ON  - SHORTEST PATH MODE");
    Serial.println("  Switch 3 ON  - RESET MEMORY");
    Serial.println("  All OFF      - IDLE/STANDBY");
    Serial.println();
    Serial.println("Note: Only turn ON one switch at a time!");
    Serial.println();
    Serial.println("Serial Commands (still available):");
    Serial.println("  Send 'R' - View saved maze from EEPROM");
    Serial.println();
    
    // Initial state reading
    currentState = readSwitchState();
    printCurrentState(currentState);
}

void loop() {
    // Read current switch state
    RobotState newState = readSwitchState();
    
    // Handle state changes
    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        handleStateChange();
    }
    
    // Check for serial commands (still available for debugging)
    if (Serial.available() > 0) {
        char command = Serial.read();
        
        if (command == 'R' || command == 'r') {
            // Retrieve and display saved maze
            Serial.println("\n>>> Retrieving saved maze from EEPROM...\n");
            mazeSolver.printSavedMaze();
            Serial.println(">>> Ready for next command...\n");
            return;
        }
    }
    
    // Execute current state behavior
    switch (currentState) {
        case IDLE:
            // Do nothing, just wait for state change
            if (isRunning) {
                isRunning = false;
                isFinished = false;
                pathFollowingMode = false;
                mazeSolver.stopMotors();
            }
            break;
            
        case EXPLORE:
            if (isRunning && !isFinished) {
                mazeSolver.runStep();
                
                // Check if exploration finished
                if (mazeSolver.isFinished()) {
                    isFinished = true;
                    isRunning = false;
                    
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║       MAZE EXPLORATION COMPLETE!       ║");
                    Serial.println("╚════════════════════════════════════════╝\n");
                    
                    // Save maze to EEPROM
                    mazeSolver.saveMazeToEEPROM();
                    
                    Serial.println(">>> Maze saved to EEPROM!");
                    Serial.println(">>> Set DIP switches to '10' for shortest path mode\n");
                }
            }
            break;
            
        case SHORTEST_PATH:
            if (isRunning && !isFinished) {
                if (!pathFollowingMode) {
                    // Compute shortest path once
                    Serial.println("\n>>> Computing shortest path...");
                    mazeSolver.computeShortestPath();
                    pathFollowingMode = true;
                    Serial.println(">>> Following shortest path...\n");
                }
                
                mazeSolver.followShortestPathStep();
                
                // Check if path following finished
                if (mazeSolver.isFinished()) {
                    isFinished = true;
                    isRunning = false;
                    pathFollowingMode = false;
                    
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║      SHORTEST PATH COMPLETE!           ║");
                    Serial.println("╚════════════════════════════════════════╝\n");
                    Serial.println(">>> Mission accomplished!\n");
                }
            }
            break;
            
        case RESET_MODE:
            // This case is handled in handleStateChange()
            break;
    }
}



// Supporting Functions
RobotState readSwitchState() {
    // Read switches (INPUT_PULLUP, so LOW = ON, HIGH = OFF)
    bool switchExplore = !digitalRead(SWITCH_EXPLORE);          // Invert because of pullup
    bool switchShortestPath = !digitalRead(SWITCH_SHORTEST_PATH); // Invert because of pullup
    bool switchReset = !digitalRead(SWITCH_RESET);              // Invert because of pullup
    
    // Priority: Reset > Explore > Shortest Path > Idle
    if (switchReset) return RESET_MODE;
    if (switchExplore) return EXPLORE;
    if (switchShortestPath) return SHORTEST_PATH;
    
    return IDLE; // All switches OFF
}

void printCurrentState(RobotState state) {
    Serial.print(">>> Current State: ");
    switch (state) {
        case IDLE:
            Serial.println("IDLE/STANDBY");
            Serial.println("    All switches OFF - Waiting...");
            break;
        case EXPLORE:
            Serial.println("EXPLORE MAZE");
            Serial.println("    Switch 1 ON - Starting exploration...");
            break;
        case SHORTEST_PATH:
            Serial.println("SHORTEST PATH");
            Serial.println("    Switch 2 ON - Following shortest path...");
            break;
        case RESET_MODE:
            Serial.println("RESET MEMORY");
            Serial.println("    Switch 3 ON - Clearing memory...");
            break;
    }
    Serial.println();
}

void handleStateChange() {
    printCurrentState(currentState);
    
    switch (currentState) {
        case IDLE:
            if (isRunning) {
                isRunning = false;
                isFinished = false;
                pathFollowingMode = false;
                mazeSolver.stopMotors();
                Serial.println(">>> Operations stopped - Now in standby");
            }
            break;
            
        case EXPLORE:
            if (!isRunning && !isFinished) {
                Serial.println(">>> Starting maze exploration in 2 seconds...");
                delay(2000);
                isRunning = true;
                isFinished = false;
                pathFollowingMode = false;
                
                // Reset robot position for fresh exploration
                mazeSolver.reset();
                Serial.println(">>> Exploration started!\n");
            }
            break;
            
        case SHORTEST_PATH:
            if (!isRunning && !isFinished) {
                Serial.println(">>> Loading saved maze from EEPROM...");
                mazeSolver.loadMazeFromEEPROM();
                
                Serial.println(">>> Starting shortest path in 2 seconds...");
                delay(2000);
                isRunning = true;
                isFinished = false;
                pathFollowingMode = false;
                
                Serial.println(">>> Shortest path mode started!\n");
            }
            break;
            
        case RESET_MODE:
            Serial.println(">>> Clearing EEPROM and resetting memory...");
            mazeSolver.reset();
            isRunning = false;
            isFinished = false;
            pathFollowingMode = false;
            
            Serial.println(">>> Memory cleared!");
            Serial.println(">>> Turn OFF Switch 3 and turn ON desired mode switch\n");
            delay(1000);
            break;
    }
}