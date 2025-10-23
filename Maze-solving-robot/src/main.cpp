#include <Arduino.h>
#include "MotorPID.h"
#include "MazeSolver.h"
#include "LineFollower.h"

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

const int irPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
const int numSensors = 8;

// Mode control
bool currentRunMode = false;  // false = Line Following, true = Maze Solving

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);
LineFollower lineFollower(leftMotor, rightMotor);

bool allWhiteDetected() {
    int sumVal = 0;
    
    for (int i = 0; i < numSensors; i++) {
        int value = digitalRead(irPins[i]);
        sumVal += value;
    }
    
    if (sumVal == 0) {
        mazeSolver.forwardForMs(80, 10);
        return false;
    } else {
        return true;
    }
}

void setup() {
    Serial.begin(9600);

    leftMotor.begin();
    rightMotor.begin();

    mazeSolver.begin();
    mazeSolver.setIRPins(irPins, numSensors);  // Share IR pins with MazeSolver
    lineFollower.begin();

    Serial.println("Robot Initialized");
    Serial.println("Starting in 2 seconds...");
    Serial.println("Mode: Line Following (will switch to Maze Solving when all white detected)");
    delay(2000);
}
// int linedetectfirst = 1;
// void loop() {
//     if (currentRunMode && !(linedetectfirst < 20)) {
//        linedetectfirst++;
//      ;
//     //    delay(300);
//         mazeSolver.forwardForMs(80, 30);
//     //    mazeSolver.readSensors();
//        mazeSolver.moveForwardWithWallFollowing();
//     //    delay(20000);

//      // Serial.println("Line following update complete");
//     } else if (linedetectfirst>=20){
//         lineFollower.update();
//     }
//     else {
    
//          // Maze solving mode - update returns true if all white detected
//         bool allWhite = mazeSolver.update();
//         if (allWhite) {
//             currentRunMode = true;  // Switch to line following mode
//             //Serial.println("Switching to Line Following Mode");
//         }
//     }
// }
bool linedetectfirst = false;
void loop() {
if (currentRunMode && !linedetectfirst){
    linedetectfirst = true;
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(80);
    rightMotor.setSpeed(80);
    leftMotor.update();
    rightMotor.update();
    delay(4000);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    // Serial.println("Line following update complete");
} 
else if (linedetectfirst){
    lineFollower.update();
} else {
         // Maze solving mode - update returns true if all white detected
        bool allWhite = mazeSolver.update();
        if (allWhite) {
            currentRunMode = true;  // Switch to line following mode
            Serial.println("Switching to Line Following Mode");
        }
    }
}

