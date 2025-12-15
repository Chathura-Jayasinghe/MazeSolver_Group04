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

#define VIEW_BUTTON 33  // Pin to trigger maze table display
#define CLEAR_BUTTON 31  // Pin to clear EEPROM

const int irPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
const int weights[] = {-3, -2, -1, 0, 0, 1, 2, 3};

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);
LineFollower lineFollower(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2, irPins, weights, 8);

bool allWhiteDetected()
{
    long sumStrength = 0;

    for (int i = 0; i < 8; i++) {
        int a = analogRead(irPins[i]);

        int strength = a - 900; // IR_THRESHOLD
        if (strength < 0) strength = 0;
        if (strength > 400) strength = 400; // MAX_STRENGTH

        sumStrength += strength;
    }

    // If all sensors detect white (low strength), return true
    return (sumStrength == 0);
}

bool turnleft()
{
    long sumStrength = 0;

    for (int i = 0; i < 6; i++) {
        int a = analogRead(irPins[i]);

        int strength = a - 900; // IR_THRESHOLD
        if (strength < 0) strength = 0;
        if (strength > 400) strength = 400; // MAX_STRENGTH

        sumStrength += strength;
    }

    return (sumStrength == 0);
}

void setup()
{
    Serial.begin(9600);

    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();
    lineFollower.setup();
    
    pinMode(VIEW_BUTTON, INPUT_PULLUP);
    pinMode(CLEAR_BUTTON, INPUT_PULLUP);

    Serial.println();
    Serial.println("FLOOD FILL MAZE SOLVER");
    Serial.println("Starting in 2 seconds...");
    delay(2000);
}

// Mode: 0 = First Maze, 1 = Line Following, 2 = Second Maze
int currentMode = 0;

void loop()
{
    static unsigned long modeStartTime = 0;

    if (currentMode == 0)
    {
        if (allWhiteDetected()) {
            float leftDist = mazeSolver.readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
            float rightDist = mazeSolver.readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

            if (leftDist < 15 && rightDist < 15)
            {
                Serial.println("All white detected with close walls! Switching to line following mode...");
                currentMode = 1;
                modeStartTime = millis(); // Record the time when switching to line-following mode
                leftMotor.setDirection(true);
                rightMotor.setDirection(true);
                leftMotor.setSpeed(60);
                rightMotor.setSpeed(60);
                delay(300);
                leftMotor.setSpeed(0);
                rightMotor.setSpeed(0);
            }
        }
        else {
            mazeSolver.runStep();
        }
    }
    else if (currentMode == 1)
    {
        lineFollower.followLine();

        if (millis() - modeStartTime > 10000)
        {
            float leftDist = mazeSolver.readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
            float rightDist = mazeSolver.readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

            if (allWhiteDetected() && (leftDist < 15 && rightDist < 15))
            {
                Serial.println("All white detected with close walls! Switching to second maze solving mode...");
                currentMode = 2;
                modeStartTime = millis(); // Record time when entering mode 2
                leftMotor.setSpeed(60);
                rightMotor.setSpeed(60);
                delay(300);
                leftMotor.setSpeed(0);
                rightMotor.setSpeed(0);
                mazeSolver.begin();
            }
        }
    }
    else if (currentMode == 2)
    {
        if (millis() - modeStartTime > 5000)
        {
            if (allWhiteDetected())
            {
                Serial.println("Endpoint detected! All white found in mode 2. Stopping robot...");
                leftMotor.setSpeed(0);
                rightMotor.setSpeed(0);
                while (true)
                {
                    delay(100000);
                }
            }
        }
        mazeSolver.runStep();
    }
    
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'C' || cmd == 'c') {
            Serial.println("\n!!! CLEARING EEPROM via Serial !!!");
            mazeSolver.clearEEPROM();
            Serial.println("EEPROM cleared! Restart robot to begin fresh.\n");
        }
        else if (cmd == 'V' || cmd == 'v') {
            Serial.println("\n--- Displaying Maze Tables ---");
            mazeSolver.displayMazeTables();
        }
    }
}