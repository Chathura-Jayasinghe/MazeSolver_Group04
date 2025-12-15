#include "LineFollower.h"

LineFollower::LineFollower(int leftMotorIn1, int leftMotorIn2, int rightMotorIn1, int rightMotorIn2, const int* irPins, const int* weights, int numSensors)
    : leftMotorIn1(leftMotorIn1), leftMotorIn2(leftMotorIn2), rightMotorIn1(rightMotorIn1), rightMotorIn2(rightMotorIn2), irPins(irPins), weights(weights), numSensors(numSensors) {}

void LineFollower::setup() {
    pinMode(leftMotorIn1, OUTPUT);
    pinMode(leftMotorIn2, OUTPUT);
    pinMode(rightMotorIn1, OUTPUT);
    pinMode(rightMotorIn2, OUTPUT);

    prevTime = millis();
}

void LineFollower::runMotors(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    analogWrite(leftMotorIn1, 0);
    analogWrite(leftMotorIn2, leftPWM);

    analogWrite(rightMotorIn1, 0);
    analogWrite(rightMotorIn2, rightPWM);
}

void LineFollower::readLineAnalog(long &weightedSum, long &sumStrength) {
    weightedSum = 0;
    sumStrength = 0;

    for (int i = 0; i < numSensors; i++) {
        int a = analogRead(irPins[i]);

        int strength = a - 1000; // IR_THRESHOLD
        if (strength < 0) strength = 0;
        if (strength > 400) strength = 400; // MAX_STRENGTH

        weightedSum += (long)weights[i] * (long)strength;
        sumStrength += strength;
    }
}

bool LineFollower::lineDetectedAnalog() {
    long ws, ss;
    readLineAnalog(ws, ss);
    return (ss >= 25); // LINE_DETECT_SUM_MIN
}

bool LineFollower::searchForLineAnalog(bool searchLeft) {
    unsigned long searchStart = millis();

    while (millis() - searchStart < 1000) { // SEARCH_DURATION_MS
        if (searchLeft) runMotors(0, 90); // TURN_SPEED_LINE
        else            runMotors(90, 0);

        delay(10);

        if (lineDetectedAnalog()) {
            return true;
        }
    }
    return false;
}

void LineFollower::followLine() {
    long weightedSum = 0;
    long sumStrength = 0;

    readLineAnalog(weightedSum, sumStrength);

    if (sumStrength < 25) { // LINE_DETECT_SUM_MIN
        if (searchForLineAnalog(true)) return;
        if (searchForLineAnalog(false)) return;
        runMotors(0, 0);
        return;
    }

    float error = (float)weightedSum / (float)sumStrength;

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;
    float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;

    float correction = Kp * error + Kd * derivative;

    int leftPWM  = baseSpeed - (int)correction;
    int rightPWM = baseSpeed + (int)correction;

    runMotors(leftPWM, rightPWM);

    prevError = error;
    prevTime  = now;
}
