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

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);

void setup() {
    Serial.begin(9600);
    
    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();

    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║     FLOOD FILL MAZE SOLVER v2.0       ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  Send 'R' - View saved maze from EEPROM");
    Serial.println("  Send 'C' - Clear EEPROM and reset");
    Serial.println("  Send 'S' - Start maze exploration");
    Serial.println();
    Serial.println("Waiting for command...");
}

void loop() {
    static bool isRunning = false;
    static bool isFinished = false;
    static bool dfsInitialized = false;
    
    // Check for serial commands
    if (Serial.available() > 0) {
        char command = Serial.read();
        
        if (command == 'R' || command == 'r') {
            // Retrieve and display saved maze
            Serial.println("\n>>> Retrieving saved maze from EEPROM...\n");
            mazeSolver.printSavedMaze();
            Serial.println(">>> Ready for next command...\n");
            return;
        }
        else if (command == 'C' || command == 'c') {
            // Clear and reset
            Serial.println("\n>>> Clearing EEPROM and resetting...\n");
            mazeSolver.reset();
            isRunning = false;
            isFinished = false;
            dfsInitialized = false;
            Serial.println(">>> System reset complete. Ready for next command...\n");
            return;
        }
        else if (command == 'S' || command == 's') {
            // Start maze exploration
            Serial.println("\n>>> Starting maze exploration in 2 seconds...\n");
            delay(2000);
            // Initialize DFS only once per run
            mazeSolver.dfsInit();
            dfsInitialized = true;

            isRunning = true;
            isFinished = false;
            return;
        }
    }
    
    // Run maze exploration if started
    if (isRunning && !isFinished) {
        mazeSolver.runStep();
        
        if (isRunning && !isFinished) {

        // Safety: make sure init happened
        if (!dfsInitialized) {
            mazeSolver.dfsInit();
            dfsInitialized = true;
        }

        // One DFS step (move or backtrack)
        mazeSolver.runStepDFS();

        // Check if DFS has fully explored all reachable cells
        if (mazeSolver.dfsFinished()) {
            isFinished = true;
            isRunning = false;

            Serial.println("\n╔════════════════════════════════════════╗");
            Serial.println("║        DFS EXPLORATION COMPLETE!       ║");
            Serial.println("╚════════════════════════════════════════╝\n");

            // Save final maze
            mazeSolver.saveMazeToEEPROM();

            Serial.println("\n>>> Maze saved to EEPROM!");
            Serial.println(">>> You can now:");
            Serial.println("    1. Disconnect the cable");
            Serial.println("    2. Reconnect anytime later");
            Serial.println("    3. Send 'R' to view the saved maze\n");
            Serial.println(">>> Ready for next command...\n");
        }
    }
}



