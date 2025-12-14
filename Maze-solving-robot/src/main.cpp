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
    Serial.println("║      MAZE NAVIGATION ROBOT v2.0       ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Starting maze navigation...");
}

void loop() {
    static bool isRunning = true;
    static bool isFinished = false;
        
    // Run maze exploration if started
    if (isRunning && !isFinished) {
        mazeSolver.runStep();
        
        // Check if finished (reached target or detected white)
        if (mazeSolver.isFinished()) {
            isFinished = true;
            isRunning = false;
            
            Serial.println("\n╔════════════════════════════════════════╗");
            Serial.println("║       MAZE EXPLORATION COMPLETE!       ║");
            Serial.println("╚════════════════════════════════════════╝\n");
        }
    }
}



