#include <Arduino.h>
#include "MotorPID.h"
#include "MazeSolver.h"
#include "LineFollower.h"

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
LineFollower lineFollower(leftMotor, rightMotor);

const int irPins[8] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};

bool allWhiteDetected()
{
    int sumVal = 0;

    for (int i = 0; i < 8; i++)
    {
        int value = digitalRead(irPins[i]);
        sumVal += value;
    }

    if (sumVal == 0)
    {
        // mazeSolver.forwardForMs(80, 10);
        return false;
    }
    else
    {
        return true;
    }
}

void setup()
{
    Serial.begin(9600);

    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();

    Serial.println();
    Serial.println("FLOOD FILL MAZE SOLVER");
    Serial.println("Starting in 2 seconds...");
    delay(2000);
}

// Mode: 0 = First Maze, 1 = Line Following, 2 = Second Maze
int currentMode = 0;

void loop()
{
    if (currentMode == 0)
    {
        // First maze solving mode
        if (allWhiteDetected())
        {
            Serial.println("All white detected! Switching to line following mode...");
            currentMode = 1;
            leftMotor.setDirection(true);
            rightMotor.setDirection(true);
            leftMotor.setSpeed(80);
            rightMotor.setSpeed(80);
            delay(500);
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            lineFollower.begin();
        }
        else
        {
            mazeSolver.runStep();
        }
    }
    else if (currentMode == 1)
    {
        // Line following mode
        if (allWhiteDetected())
        {
            Serial.println("All white detected at end of line! Switching to second maze...");
            currentMode = 2;
            leftMotor.setSpeed(80);
            rightMotor.setSpeed(80);
            delay(500);
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            mazeSolver.begin();
        }
        else
        {
            lineFollower.update();
        }
    }
    else if (currentMode == 2)
    {
        // Second maze solving mode
        mazeSolver.runStep();
    }
}