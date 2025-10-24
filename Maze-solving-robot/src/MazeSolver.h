#ifndef MAZESOLVER_H
#define MAZESOLVER_H

#include <Arduino.h>
#include "MotorPID.h"

// Ultrasonic sensor pins
#define US_FRONT_TRIG 22
#define US_FRONT_ECHO 24
#define US_LEFT_TRIG 26
#define US_LEFT_ECHO 28
#define US_RIGHT_TRIG 37
#define US_RIGHT_ECHO 36

// Control parameters
#define OBSTACLE_DISTANCE 8
#define BASE_SPEED 80
#define MAX_CORRECTION 40
#define ALIGNMENT_THRESHOLD 1.0

// Maze thresholds
#define FRONT_STOP_CM   OBSTACLE_DISTANCE
#define SIDE_OPEN_CM    15
#define SIDE_WALL_CM    15

// Turn parameters
#define COUNTS_PER_360  444L               
#define COUNTS_PER_90   270L //250 - CORRECT FOR YOUR ROBOT
#define TURN_SPEED    100 

struct RangeReadings {
    float front_cm = -1.0f;
    float left_cm  = -1.0f;
    float right_cm = -1.0f;
};

enum RobotState {
    MOVING_FORWARD,
    STOPPED,
};

enum JunctionType {
    JT_STRAIGHT,
    JT_L_LEFT,
    JT_L_RIGHT,
    JT_T,
    JT_CROSS_OR_CORNER,
    JT_DEAD_END
};

enum Decision {
    DEC_LEFT,
    DEC_STRAIGHT,
    DEC_RIGHT,
    DEC_UTURN,
    DEC_NONE
};

class MazeSolver {
public:
    MazeSolver(MotorPID& leftMotor, MotorPID& rightMotor);
    void begin();
    void setIRPins(const int* pins, int count);  // Set IR sensor pins from external source
    bool update();  // Returns true if all IR sensors detect white
    RobotState getCurrentState() { return currentState; }
    void setCurrentState(RobotState state) { currentState = state; }
    void forwardForMs(int pwmBase, long targetPulses);
     void moveForwardWithWallFollowing();
     void readSensors();


private:
    MotorPID& leftMotor;
    MotorPID& rightMotor;
    
    RangeReadings ranges;
    RobotState currentState;
    
    // IR sensor pins (shared with LineFollower)
    const int* irPins;
    int irPinCount;
    
    // PD controller variables
    float lastError;
    unsigned long lastUpdateTime;
    const float TARGET_DIST_CM     = 6.0f;
    const float MAX_DETECT_DIST    = 80.0f;
    const float MIN_VALID_DIST     = 5.0f;
    const float Kp                 = 3.0f;
    const float Kd                 = 0.0f;

    // Helper functions
    float ultrasonic_sensor_distance(int trigPin, int echoPin);
    // void readSensors();
    bool allWhiteDetected();  // Check if all IR sensors detect white
    bool sideIsOpen(float d);
    long encLeft();
    long encRight();
    void encZeroBoth();
    
    // Movement functions
    void calculateWallFollowingSpeeds(int &leftSpeed, int &rightSpeed);
    // void moveForwardWithWallFollowing();
    void moveForward();
    void stopMotors();
    void brakeShort();
    void correctionRotate();
    void Turn90(int dir);
    void TurnWithPulse(int counts, int dir);
    void rotateLeft90();
    void rotateRight90();
    void reverseMotors(int duration_ms);
    void rotateUTurn();
    
    // Decision making
    JunctionType classifyJunction(const RangeReadings& r);
    Decision decideAction(JunctionType jt);
    void executeDecision(Decision d);
};

#endif // MAZESOLVER_H
