#include <Arduino.h>
#include "MotorPID.h"

// Ultrasonic sensor pins
#define US_FRONT_TRIG 22
#define US_FRONT_ECHO 24
#define US_LEFT_TRIG 26
#define US_LEFT_ECHO 28
#define US_RIGHT_TRIG 37
#define US_RIGHT_ECHO 36

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

// Motor specifications
#define PPR 11
#define GEAR_RATIO 21
#define PULSES_PER_REV (PPR * GEAR_RATIO)

// Control parameters
#define OBSTACLE_DISTANCE 10
#define BASE_SPEED 50
#define MAX_CORRECTION 40
#define ALIGNMENT_THRESHOLD 1.0
#define CORRECTION_GAIN 2.0

// ===== Maze thresholds (tune on your maze) =====
#define FRONT_STOP_CM   OBSTACLE_DISTANCE
#define SIDE_OPEN_CM    25
#define SIDE_WALL_CM    10
  

// ===== Turn timings (timed turns; tune on your floor) =====
#define TURN_SPEED        90
#define TURN_90_MS        420
#define UTURN_MS          (TURN_90_MS * 2)
#define BRAKE_MS          120
#define POST_TURN_FWD_MS  150


// Motor objects
MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, false);

// Quick helpers
inline bool sideIsOpen(float d) { return d >= SIDE_OPEN_CM && d > 0; }
inline bool sideIsWall(float d) { return d > 0 && d <= SIDE_WALL_CM; }

struct RangeReadings
{
    float front_cm = -1.0f;
    float left_cm  = -1.0f;
    float right_cm = -1.0f;
} ranges;

enum RobotState {
    MOVING_FORWARD,
    STOPPED,
    OBSTACLE_DETECTED
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


JunctionType classifyJunction(const RangeReadings& r) {
    const bool frontBlocked = (r.front_cm > 0 && r.front_cm < FRONT_STOP_CM);
    const bool leftOpen     = sideIsOpen(r.left_cm);
    const bool rightOpen    = sideIsOpen(r.right_cm);
    const bool leftWall     = sideIsWall(r.left_cm);
    const bool rightWall    = sideIsWall(r.right_cm);

    if (frontBlocked) {
        if (leftOpen && rightOpen)  return JT_T;
        if (leftOpen && !rightOpen) return JT_L_LEFT;
        if (rightOpen && !leftOpen) return JT_L_RIGHT;
        return JT_DEAD_END;
    } else {
        if (leftOpen || rightOpen)  return JT_CROSS_OR_CORNER;
        return JT_STRAIGHT;
    }
}


Decision decideAction(JunctionType jt) {
    // Left-hand rule:
    switch (jt) {
        case JT_T:                return DEC_LEFT;
        case JT_L_LEFT:           return DEC_LEFT;
        case JT_CROSS_OR_CORNER:  return DEC_LEFT;
        case JT_STRAIGHT:         return DEC_STRAIGHT;
        case JT_L_RIGHT:          return DEC_RIGHT;
        case JT_DEAD_END:         return DEC_UTURN;
        default:                  return DEC_NONE;
    }
}


RobotState currentState = STOPPED;

float ultrasonic_sensor_distance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 15000UL); // 15ms timeout
    if (duration == 0) {
        return -1.0;
    }
    float distance_cm = duration * 0.0343 / 2.0;
    return distance_cm;
}

void setup() {
    Serial.begin(9600);

    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);

    // Initialize motors
    leftMotor.begin();
    rightMotor.begin();

    Serial.println("Maze Robot Initialized");
    Serial.println("Starting in 2 seconds...");
    delay(2000);

    currentState = MOVING_FORWARD;
}

void readSensors() {
    ranges.front_cm = ultrasonic_sensor_distance(US_FRONT_TRIG, US_FRONT_ECHO);
    delay(10);
    ranges.left_cm = ultrasonic_sensor_distance(US_LEFT_TRIG, US_LEFT_ECHO);
    delay(10);
    ranges.right_cm = ultrasonic_sensor_distance(US_RIGHT_TRIG, US_RIGHT_ECHO);
    delay(10);
}

// Function to calculate motor speeds for wall following
void calculateWallFollowingSpeeds(int &leftSpeed, int &rightSpeed) {
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED;
    
    if (ranges.left_cm > 0 && ranges.right_cm > 0) {
        float difference = ranges.left_cm - ranges.right_cm;
        
        if (fabsf(difference) > ALIGNMENT_THRESHOLD) {
            float correction = difference * CORRECTION_GAIN;
            correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
            
            if (difference > 0) {
                leftSpeed = BASE_SPEED - fabsf(correction);
                rightSpeed = BASE_SPEED + fabsf(correction) * 0.3;
            } else {
                rightSpeed = BASE_SPEED - fabsf(correction);
                leftSpeed = BASE_SPEED + fabsf(correction) * 0.3;
            }
        
            leftSpeed  = constrain((int)(BASE_SPEED - correction), 0, 255);
            rightSpeed = constrain((int)(BASE_SPEED + correction), 0, 255);
        }
    } else if (ranges.left_cm > 0) {
        if (ranges.left_cm < 15) {
            rightSpeed = BASE_SPEED - 30;
            leftSpeed = BASE_SPEED + 10;
        } else if (ranges.left_cm > 25) {
            leftSpeed = BASE_SPEED - 30;
            rightSpeed = BASE_SPEED + 10;
        }
    } else if (ranges.right_cm > 0) {
        if (ranges.right_cm < 15) {
            leftSpeed = BASE_SPEED - 30;
            rightSpeed = BASE_SPEED + 10;
        } else if (ranges.right_cm > 25) {
            rightSpeed = BASE_SPEED - 30;
            leftSpeed = BASE_SPEED + 10;
        }
    }
}

// Function to move forward with wall following
void moveForwardWithWallFollowing() {
    int leftSpeed, rightSpeed;
    calculateWallFollowingSpeeds(leftSpeed, rightSpeed);
    
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

// Function to move forward
void moveForward() {
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);
}

// Function to stop motors
void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

// Function to check for obstacles
bool isObstacleAhead() {
    if (ranges.front_cm > 0 && ranges.front_cm < OBSTACLE_DISTANCE) {
        return true;
    }
    return false;
}

void brakeShort() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(BRAKE_MS);
}

void forwardForMs(int pwm, int ms) {
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(pwm);
    rightMotor.setSpeed(pwm);
    delay(ms);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void rotateLeft90() {
    brakeShort();
    leftMotor.setDirection(false);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    delay(TURN_90_MS);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void rotateRight90() {
    brakeShort();
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    delay(TURN_90_MS);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void rotateUTurn() {
    brakeShort();
    leftMotor.setDirection(false);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    delay(UTURN_MS);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void executeDecision(Decision d) {
    switch (d) {
        case DEC_LEFT:     rotateLeft90();     forwardForMs(BASE_SPEED, POST_TURN_FWD_MS); break;
        case DEC_STRAIGHT: forwardForMs(BASE_SPEED, POST_TURN_FWD_MS);                     break;
        case DEC_RIGHT:    rotateRight90();    forwardForMs(BASE_SPEED, POST_TURN_FWD_MS); break;
        case DEC_UTURN:    rotateUTurn();      forwardForMs(BASE_SPEED, POST_TURN_FWD_MS); break;
        default:           break;
    }
}


void loop() {
    readSensors();

    Serial.print("Front: ");
    Serial.print(ranges.front_cm);
    Serial.print(" cm, Left: ");
    Serial.print(ranges.left_cm);
    Serial.print(" cm, Right: ");
    Serial.print(ranges.right_cm);
    Serial.println(" cm");

    switch (currentState) {
    case MOVING_FORWARD: {
        JunctionType jt = classifyJunction(ranges);

        const bool mustDecide =
        (jt == JT_T) || (jt == JT_L_LEFT) || (jt == JT_L_RIGHT) ||
        (jt == JT_DEAD_END) || (ranges.front_cm > 0 && ranges.front_cm < FRONT_STOP_CM);

        if (mustDecide) {
        stopMotors();
        Decision d = decideAction(jt);
        executeDecision(d);
        currentState = MOVING_FORWARD;
        break;
        }

        if (jt == JT_CROSS_OR_CORNER) {
        moveForwardWithWallFollowing();
        break;
        }

        moveForwardWithWallFollowing();
        break;
    }

    case OBSTACLE_DETECTED:
        stopMotors();
        executeDecision(decideAction(classifyJunction(ranges)));
        currentState = MOVING_FORWARD;
        break;

    case STOPPED:
        stopMotors();
        break;
    }

    leftMotor.update();
    rightMotor.update();

    delay(50);
}
