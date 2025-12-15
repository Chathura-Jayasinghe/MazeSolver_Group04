#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>

// IR Analog Pins (A8..A15)
#define IR1 A8
#define IR2 A9
#define IR3 A10
#define IR4 A11
#define IR5 A12
#define IR6 A13
#define IR7 A14
#define IR8 A15

class LineFollower {
public:
    LineFollower(int leftMotorIn1, int leftMotorIn2, int rightMotorIn1, int rightMotorIn2, const int* irPins, const int* weights, int numSensors);
    void setup();
    void followLine();

private:
    void runMotors(int leftPWM, int rightPWM);
    void readLineAnalog(long &weightedSum, long &sumStrength);
    bool lineDetectedAnalog();
    bool searchForLineAnalog(bool searchLeft);

    int leftMotorIn1, leftMotorIn2, rightMotorIn1, rightMotorIn2;
    const int* irPins;
    const int* weights;
    int numSensors;

    float Kp = 35.0;
    float Kd = 0.0;
    int baseSpeed = 80;

    float prevError = 0;
    unsigned long prevTime = 0;
};

#endif