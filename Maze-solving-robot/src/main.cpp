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

// Control parameters
#define OBSTACLE_DISTANCE 8
#define BASE_SPEED 100
#define MAX_CORRECTION 40
#define ALIGNMENT_THRESHOLD 1.0
#define Kp 2.0
#define Kd 1.0
float lastError = 0.0f;
unsigned long lastUpdateTime = 0;

// ===== Maze thresholds (tune on your maze) =====
#define FRONT_STOP_CM   OBSTACLE_DISTANCE
#define SIDE_OPEN_CM    15
#define SIDE_WALL_CM    15

// Turn parameters
#define COUNTS_PER_360  444L               
#define COUNTS_PER_90   280
#define TURN_SPEED    100 
             

// Motor objects
MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

inline bool sideIsOpen(float d) { return d >= SIDE_OPEN_CM && d > 0; }
inline long encLeft()  { return leftMotor.getEncoderCount(); }   
inline long encRight() { return rightMotor.getEncoderCount(); } 
inline void encZeroBoth() {
    leftMotor.resetEncoder();   
    rightMotor.resetEncoder();  
}

struct RangeReadings{
    float front_cm = -1.0f;
    float left_cm  = -1.0f;
    float right_cm = -1.0f;
} ranges;

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


JunctionType classifyJunction(const RangeReadings& r) {
    const bool frontBlocked = (r.front_cm > 0 && r.front_cm < FRONT_STOP_CM);
    const bool leftOpen     = sideIsOpen(r.left_cm);
    const bool rightOpen    = sideIsOpen(r.right_cm);

    if (leftOpen && !rightOpen)  return JT_L_LEFT;
    else if (!leftOpen && rightOpen)  return JT_L_RIGHT;
    else if (leftOpen && rightOpen)   return frontBlocked ? JT_T : JT_CROSS_OR_CORNER;
    else                             return frontBlocked ? JT_DEAD_END : JT_STRAIGHT;
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

    unsigned long duration = pulseIn(echoPin, HIGH, 10000UL); // 10ms timeout
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


void calculateWallFollowingSpeeds(int &leftSpeed, int &rightSpeed) {
    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;

    if (dt <= 0) dt = 0.01f;  // Prevent division by zero
    lastUpdateTime = now;

    // Calculate error between left and right distances
    float error = ranges.left_cm - ranges.right_cm;

    if (fabsf(error) < ALIGNMENT_THRESHOLD) error = 0.0f;

    // if (error>25.0f && error<50.0f){
    //     leftSpeed  = BASE_SPEED - 40;
    //     rightSpeed = BASE_SPEED + 40;
    //     delay(50);
    //     leftMotor.setSpeed(0);
    //     rightMotor.setSpeed(0);
    //     return; 
    // }
    // if(error<-25.0f && error>-50.0f){
    //     leftSpeed  = BASE_SPEED + 40;
    //     rightSpeed = BASE_SPEED - 40;
    //     delay(50);
    //     leftMotor.setSpeed(0);
    //     rightMotor.setSpeed(0);
    //     return; 
    // }

    // ---- PD Control ----
    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = (Kp * error) + (Kd * derivative);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

    // ---- Motor Speed Adjustment ----
    leftSpeed  = constrain(BASE_SPEED - correction, 0, 255);
    rightSpeed = constrain(BASE_SPEED + correction, 0, 255);
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

void brakeShort() {
    stopMotors();
    delay(500);
}


void forwardForMs(int pwmBase, long targetPulses) {
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);

    encZeroBoth();

    leftMotor.setSpeed(pwmBase);
    rightMotor.setSpeed(pwmBase);

    while (true) {
        long cL = labs(encLeft());
        long cR = labs(encRight());
        long avg = (cL + cR) >> 1;

        if (avg >= targetPulses) break;  

        delayMicroseconds(700);        
    }

    stopMotors();
    encZeroBoth();  
}


void Turn90(int dir) {
    const long target = COUNTS_PER_90; 
    const bool turnCW  = (dir > 0);
    const bool turnCCW = (dir < 0);

    leftMotor.setDirection(turnCCW); 
    rightMotor.setDirection(turnCW); 

    encZeroBoth();

    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);

    while (true) {
        // Read both encoders once
        long cL = labs(encLeft());
        long cR = labs(encRight());

        long avg = (cL + cR) >> 1;
        if (avg >= target) break;  

        delayMicroseconds(800);
    }

    stopMotors();
    encZeroBoth(); 
}

void rotateLeft90()  { Turn90(+1); }
void rotateRight90() { Turn90(-1); }

void rotateUTurn() {

    rotateLeft90();
    delay(100);
    rotateLeft90();
}

void executeDecision(Decision d) {
  switch (d) {
        case DEC_LEFT:     forwardForMs(BASE_SPEED, 300);; rotateLeft90(); forwardForMs(BASE_SPEED, 200);  break;
        case DEC_STRAIGHT: break;
        case DEC_RIGHT:    forwardForMs(BASE_SPEED, 300); ; rotateRight90();  forwardForMs(BASE_SPEED, 200); break;
        case DEC_UTURN:    rotateUTurn();     break;
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
            Serial.println(jt);

            const bool mustDecide =
            (jt == JT_T) || (jt == JT_L_LEFT) || (jt == JT_L_RIGHT) ||
            (jt == JT_DEAD_END) || (ranges.front_cm > 0 && ranges.front_cm < FRONT_STOP_CM);

            if (mustDecide) {
                stopMotors();

                Decision d = decideAction(jt);
                Serial.println(d);

                executeDecision(d);
                currentState = MOVING_FORWARD;
                break;
            }
            moveForwardWithWallFollowing();
            break;
        }
        case STOPPED:
            stopMotors();
            break;
    }

    leftMotor.update();
    rightMotor.update();
}



