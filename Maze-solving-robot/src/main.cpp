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
#define OBSTACLE_DISTANCE 8
#define BASE_SPEED 50
#define MAX_CORRECTION 40
#define ALIGNMENT_THRESHOLD 1.0
#define CORRECTION_GAIN 2.0

// ===== Maze thresholds (tune on your maze) =====
#define FRONT_STOP_CM   OBSTACLE_DISTANCE
#define SIDE_OPEN_CM    15
#define SIDE_WALL_CM    15
  

// ===== Turn timings (timed turns; tune on your floor) =====
#define TURN_SPEED        90
#define TURN_90_MS        400
#define UTURN_MS          1000
#define BRAKE_MS          500
#define POST_TURN_FWD_MS  1000

#define COUNTS_PER_360  444L               // Adjust based on your robot's turning characteristics
#define COUNTS_PER_90   (COUNTS_PER_360 / 2)

#define TURN_PWM_FAST    110               // main spin speed
#define TURN_PWM_SLOW    80                // used to let the lagging side catch up
#define TURN_MIN_PWM     60                // avoid stalling brushed DC motors
#define TURN_EPS         8L                // allowed per-wheel count mismatch


// Motor objects
MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

// Quick helpers
inline bool sideIsOpen(float d) { return d >= SIDE_OPEN_CM && d > 0; }
inline long encLeft()  { return leftMotor.getEncoderCount(); }   // adjust if needed
inline long encRight() { return rightMotor.getEncoderCount(); }  // adjust if needed
inline void encZeroBoth() {
    leftMotor.resetEncoder();   // adjust if needed
    rightMotor.resetEncoder();  // adjust if needed
}

struct RangeReadings
{
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

// Function to calculate motor speeds for wall following
void calculateWallFollowingSpeeds(int &leftSpeed, int &rightSpeed) {
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED;
    
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

// dir = +1 => left turn (CCW):  left backward,  right forward
// dir = -1 => right turn (CW):  left forward,   right backward
void encoderTurn90_noKP(int dir)
{
    const long target = COUNTS_PER_90;

    // Set spin directions
    leftMotor.setDirection(dir < 0);   // CW: left fwd; CCW: left back
    rightMotor.setDirection(dir > 0);  // CW: right back; CCW: right fwd

    // Zero baselines
    encZeroBoth();

    // Start both sides at FAST
    int pwmL = TURN_PWM_FAST;
    int pwmR = TURN_PWM_FAST;
    leftMotor.setSpeed(pwmL);
    rightMotor.setSpeed(pwmR);

    for (;;)
    {
        // keep motor drivers/filters fresh
        leftMotor.update();
        rightMotor.update();

        long cL = labs(encLeft());
        long cR = labs(encRight());
        long avg = (cL + cR) >> 1;

        if (avg >= target) break;

        long err = cL - cR;  // + => left is ahead, - => right is ahead

        int wantL = TURN_PWM_FAST;
        int wantR = TURN_PWM_FAST;

        if (err > TURN_EPS) {
            // left ahead → slow left a bit to let right catch up
            wantL = TURN_PWM_SLOW;
            wantR = TURN_PWM_FAST;
        } else if (err < -TURN_EPS) {
            // right ahead → slow right
            wantL = TURN_PWM_FAST;
            wantR = TURN_PWM_SLOW;
        }
        // safety: avoid stall
        wantL = constrain(wantL, TURN_MIN_PWM, 255);
        wantR = constrain(wantR, TURN_MIN_PWM, 255);

        if (wantL != pwmL) { pwmL = wantL; leftMotor.setSpeed(pwmL); }
        if (wantR != pwmR) { pwmR = wantR; rightMotor.setSpeed(pwmR); }
        // no delays — loop continues until encoder target reached
    }

    // stop hard
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    // Optional tiny square-up (fixed nudge, still no KP)
    long cL = labs(encLeft()), cR = labs(encRight());
    if (labs(cL - cR) > TURN_EPS) {
        bool nudgeLeft = (cL < cR);
        if (nudgeLeft) {
            leftMotor.setSpeed(max(70, TURN_MIN_PWM));
            leftMotor.update();
            long base = labs(encLeft());
            while (labs(labs(encLeft()) - base) < 2) { leftMotor.update(); rightMotor.update(); }
            leftMotor.setSpeed(0);
        } else {
            rightMotor.setSpeed(max(70, TURN_MIN_PWM));
            rightMotor.update();
            long base = labs(encRight());
            while (labs(labs(encRight()) - base) < 2) { leftMotor.update(); rightMotor.update(); }
            rightMotor.setSpeed(0);
        }
    }

    encZeroBoth(); // clean slate for next move
}

// Convenience wrappers
void rotateLeft90()  { encoderTurn90_noKP(+1); }
void rotateRight90() { encoderTurn90_noKP(-1); }

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
    // Serial.print("******Executing decision: ****");
    switch (d) {
        case DEC_LEFT:     forwardForMs(BASE_SPEED, 1500); rotateLeft90();     forwardForMs(BASE_SPEED, POST_TURN_FWD_MS); break;
        case DEC_STRAIGHT: forwardForMs(BASE_SPEED, POST_TURN_FWD_MS);                     break;
        case DEC_RIGHT:    forwardForMs(BASE_SPEED, 1500); rotateRight90();    forwardForMs(BASE_SPEED, POST_TURN_FWD_MS); break;
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
            // Serial.print("****Classified junction: ");
            Serial.println(jt);

            const bool mustDecide =
            (jt == JT_T) || (jt == JT_L_LEFT) || (jt == JT_L_RIGHT) ||
            (jt == JT_DEAD_END) || (ranges.front_cm > 0 && ranges.front_cm < FRONT_STOP_CM);

            if (mustDecide) {
                stopMotors();

                Decision d = decideAction(jt);
                // Serial.print("****Decided on action: ");
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
