#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include "MotorPID.h"

// IR Sensor pins
#define IR1 43
#define IR2 44
#define IR3 45
#define IR4 46
#define IR5 47
#define IR6 48
#define IR7 49
#define IR8 50

// Line Following Parameters
#define NUM_SENSORS 8
#define BASE_SPEED_LINE 80
#define TURN_SPEED_LINE 60
#define SEARCH_DURATION_MS 300

// Turn Search State Machine
enum TurnSearchState_Line {
    NORMAL_FOLLOWING,
    SEARCHING_LEFT,
    SEARCHING_RIGHT,
    TURN_FOUND
};

class LineFollower {
public:
    LineFollower(MotorPID& leftMotor, MotorPID& rightMotor);
    void begin();
    void update();
    
    // IR sensor functions (made public for reuse)
    bool lineDetected();

private:
    MotorPID& leftMotor;
    MotorPID& rightMotor;
    
    // IR Sensors
    const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
    const int weights[NUM_SENSORS] = {-3, -2, -1, 0, 0, 1, 2, 3};
    
    // PD Control parameters
    const float Kp_line = 30.0;
    const float Kd_line = 0.0;
    float prevError;
    unsigned long prevTime;
    
    // State machine
    TurnSearchState_Line turnState;
    
    // Helper functions
    void runMotors_Line(int leftPWM, int rightPWM);
    void turnLeft_line();
    void turnRight_Line();
    void stopMotors();
    bool searchForLine(bool searchLeft);
};

#endif // LINEFOLLOWER_H
