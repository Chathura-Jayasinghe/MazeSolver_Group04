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

    Serial.println("FLOOD FILL MAZE SOLVER");
    Serial.println("Starting in 2 seconds...");
    delay(2000);
}

void loop() {
    // --- Mode selection (temporary booleans; replace with switch later) ---
    static bool modeScan = true;    // Mode 1: explore + build map until IR white
    static bool modeFollow = false; // Mode 2: follow precomputed shortest path
    static bool modeReset = false;  // Mode 3: clear memory and pose

    // Example toggling logic placeholder:
    // You can change these booleans at runtime based on a physical switch later.

    // if (modeReset) {
    //     mazeSolver.reset();
    //     modeReset = false;
    //     modeScan = true;
    //     modeFollow = false;
    //     Serial.println("RESET DONE");
    //     delay(500);
    //     return;
    // }

    if (modeScan) {
        if (true) {
            mazeSolver.runStep();
        } else {
            Serial.println("TARGET DETECTED (IR or coords). Computing path...");
            mazeSolver.computeShortestPath();
            // switch to follow mode
            // modeScan = false;
            // modeFollow = true;
            Serial.println("PATH READY. Switching to follow mode.");
            delay(500);
        }
        return;
    }

    // if (modeFollow) {
    //     // Follow the stored shortest path one step at a time
    //     mazeSolver.followShortestPathStep();
    //     // When path done, stop
    //     if (mazeSolver.isFinished()) {
    //         Serial.println("TARGET REACHED - FOLLOW MODE COMPLETE");
    //         delay(1000);
    //     }
    //     return;
    // }
}