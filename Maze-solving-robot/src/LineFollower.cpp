#include "LineFollower.h"

LineFollower::LineFollower(MotorPID& left, MotorPID& right)
    : leftMotor(left), rightMotor(right), 
      prevError(0.0), prevTime(0), turnState(NORMAL_FOLLOWING) {
}

void LineFollower::begin() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(irPins[i], INPUT);
    }
    
    turnState = NORMAL_FOLLOWING;
}

void LineFollower::runMotors_Line(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(leftPWM);
    rightMotor.setSpeed(rightPWM);
}

void LineFollower::turnLeft_line() {
    leftMotor.setDirection(false);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED_LINE);
    rightMotor.setSpeed(TURN_SPEED_LINE);
}

void LineFollower::turnRight_Line() {
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED_LINE);
    rightMotor.setSpeed(TURN_SPEED_LINE);
}

void LineFollower::stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

bool LineFollower::lineDetected() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (digitalRead(irPins[i]) == HIGH) {
            return true;
        }
    }
    return false;
}

bool LineFollower::searchForLine(bool searchLeft) {
    unsigned long searchStart = millis();
    
    while (millis() - searchStart < SEARCH_DURATION_MS) {
        if (searchLeft) {
            turnLeft_line();
        } else {
            turnRight_Line();
        }

        delay(10);
        
        if (lineDetected()) {
            return true;
        }
    }
    return false;
}

void LineFollower::update() {
    int sumVal = 0;
    int weightedSum = 0;

    // Read IR sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
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

            float correction = Kp_line * error + Kd_line * derivative;

            int leftPWM = BASE_SPEED_LINE - correction;
            int rightPWM = BASE_SPEED_LINE + correction;

            runMotors_Line(leftPWM, rightPWM);

            prevError = error;
            prevTime = now;
            break;
        }

        case SEARCHING_LEFT: {
            if (searchForLine(true)) {
                turnState = NORMAL_FOLLOWING;
            } else {
                turnState = SEARCHING_RIGHT;
                searchForLine(false);
                delay(100);
            }
            break;
        }

        case SEARCHING_RIGHT: {
            if (searchForLine(false)) {
                turnState = NORMAL_FOLLOWING;
            } else {
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
