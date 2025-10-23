#include <Arduino.h>

// IR Sensor Pins
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

// Motor Control Pins
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 8
#define RIGHT_MOTOR_IN1 6
#define RIGHT_MOTOR_IN2 5

// Line Following Parameters
const int weights[numSensors] = {-3, -2, -1, 0, 0, 1, 2, 3};
const float Kp = 30.0;
const float Kd = 0.0;
const int baseSpeed = 80;

// Control Variables
float prevError = 0.0;
unsigned long prevTime = 0;

// Turn Search State Machine
enum TurnSearchState {
    NORMAL_FOLLOWING,
    SEARCHING_LEFT,
    SEARCHING_RIGHT,
    TURN_FOUND
};

TurnSearchState turnState = NORMAL_FOLLOWING;
const int SEARCH_DURATION_MS = 300;
const int TURN_SPEED = 60;

void setup() {
    Serial.begin(9600);
    
    // Initialize IR sensor pins
    for (int i = 0; i < numSensors; i++) {
        pinMode(irPins[i], INPUT);
    }
    
    // Initialize motor pins
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
}

void runMotors(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    analogWrite(LEFT_MOTOR_IN1, leftPWM);
    analogWrite(LEFT_MOTOR_IN2, 0);
    analogWrite(RIGHT_MOTOR_IN1, rightPWM);
    analogWrite(RIGHT_MOTOR_IN2, 0);
}

void turnLeft() {
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_IN1, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_IN2, 0);
}

void turnRight() {
    analogWrite(LEFT_MOTOR_IN1, TURN_SPEED);
    analogWrite(LEFT_MOTOR_IN2, 0);
    analogWrite(RIGHT_MOTOR_IN1, 0);
    analogWrite(RIGHT_MOTOR_IN2, TURN_SPEED);
}

void stopMotors() {
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, 0);
    analogWrite(RIGHT_MOTOR_IN1, 0);
    analogWrite(RIGHT_MOTOR_IN2, 0);
}

bool lineDetected() {
    for (int i = 0; i < numSensors; i++) {
        if (digitalRead(irPins[i]) == HIGH) {
            return true;
        }
    }
    return false;
}

bool searchForLine(bool searchLeft) {
    unsigned long searchStart = millis();
    
    while (millis() - searchStart < SEARCH_DURATION_MS) {
        if (searchLeft) {
            turnLeft();
        } else {
            turnRight();
        }
        
        delay(10);
        
        if (lineDetected()) {
            return true;
        }
    }
    
    return false;
}

void loop() {
    int sumVal = 0;
    int weightedSum = 0;

    // Read IR sensors
    for (int i = 0; i < numSensors; i++) {
        int value = digitalRead(irPins[i]);
        if (value == HIGH) {
            weightedSum += weights[i];
            sumVal++;
        }
    }

    // Handle different states
    switch (turnState) {
        case NORMAL_FOLLOWING: {
            if (sumVal == 0) {
                Serial.println("Line lost - starting L-turn search");
                turnState = SEARCHING_LEFT;
                return;
            }

            // Normal line following with PD control
            float error = (float)weightedSum / sumVal;

            unsigned long now = millis();
            float dt = (now - prevTime) / 1000.0;
            float derivative = 0;
            if (dt > 0.0) {
                derivative = (error - prevError) / dt;
            }

            float correction = Kp * error + Kd * derivative;

            int leftPWM = baseSpeed - correction;
            int rightPWM = baseSpeed + correction;

            runMotors(leftPWM, rightPWM);

            prevError = error;
            prevTime = now;
            break;
        }

        case SEARCHING_LEFT: {
            Serial.println("Searching left...");
            if (searchForLine(true)) {
                Serial.println("Found line on the left!");
                turnState = NORMAL_FOLLOWING;
            } else {
                Serial.println("Line not found on left, trying right...");
                turnState = SEARCHING_RIGHT;
                searchForLine(false);
                delay(100);
            }
            break;
        }

        case SEARCHING_RIGHT: {
            Serial.println("Searching right...");
            if (searchForLine(false)) {
                Serial.println("Found line on the right!");
                turnState = NORMAL_FOLLOWING;
            } else {
                Serial.println("Line not found in either direction - stopping");
                stopMotors();
                delay(1000);
                turnState = NORMAL_FOLLOWING;
            }
            break;
        }

        default:
            turnState = NORMAL_FOLLOWING;
            break;
    }

    delay(20);
}
