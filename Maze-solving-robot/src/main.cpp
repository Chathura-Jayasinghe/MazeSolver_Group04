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
#define OBSTACLE_DISTANCE 10      // Stop if front obstacle within 5cm
#define BASE_SPEED 50            // Base forward speed (0-255)
#define MAX_CORRECTION 60         // Maximum speed correction for alignment
#define ALIGNMENT_THRESHOLD 1.0   // Dead zone in cm (don't correct if difference < 1cm)
#define CORRECTION_GAIN 3.0       // How aggressively to correct (higher = more aggressive)
  

// Motor objects
MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, false);

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

RobotState currentState = STOPPED;

float ultrasonic_sensor_distance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL); // 25ms timeout
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
  
  // Check if both left and right sensors have valid readings
  if (ranges.left_cm > 0 && ranges.right_cm > 0) {
    float difference = ranges.left_cm - ranges.right_cm;
    
    // Only correct if outside the dead zone
    if (abs(difference) > ALIGNMENT_THRESHOLD) {
      float correction = difference * CORRECTION_GAIN;
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      
      if (difference > 0) {
        // Too close to left wall - turn right
        leftSpeed = BASE_SPEED - abs(correction);
        rightSpeed = BASE_SPEED + abs(correction) * 0.3;
      } else {
        // Too close to right wall - turn left
        rightSpeed = BASE_SPEED - abs(correction);
        leftSpeed = BASE_SPEED + abs(correction) * 0.3;
      }
      
      // Ensure speeds stay within valid range
      leftSpeed = constrain(leftSpeed, BASE_SPEED - MAX_CORRECTION, 255);
      rightSpeed = constrain(rightSpeed, BASE_SPEED - MAX_CORRECTION, 255);
    }
  } 
  // If only one sensor has valid reading, use it for guidance
  else if (ranges.left_cm > 0) {
    // Only left sensor active - try to maintain distance from left wall
    if (ranges.left_cm < 15) {
      // Too close to left - turn right
      rightSpeed = BASE_SPEED - 30;
      leftSpeed = BASE_SPEED + 10;
    } else if (ranges.left_cm > 25) {
      // Too far from left - turn left
      leftSpeed = BASE_SPEED - 30;
      rightSpeed = BASE_SPEED + 10;
    }
  }
  else if (ranges.right_cm > 0) {
    // Only right sensor active - try to maintain distance from right wall
    if (ranges.right_cm < 15) {
      // Too close to right - turn left
      leftSpeed = BASE_SPEED - 30;
      rightSpeed = BASE_SPEED + 10;
    } else if (ranges.right_cm > 25) {
      // Too far from right - turn right
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
  leftMotor.setDirection(true);   // Forward
  rightMotor.setDirection(true);  // Forward
  leftMotor.setSpeed(50);        // Adjust speed as needed (0-255)
  rightMotor.setSpeed(50);
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

void loop() {
  readSensors();

  Serial.print("Front: ");
  Serial.print(ranges.front_cm);
  Serial.print(" cm, Left: ");
  Serial.print(ranges.left_cm);
  Serial.print(" cm, Right: ");
  Serial.print(ranges.right_cm);
  Serial.println(" cm");

  // State machine for robot behavior
  switch (currentState) {
    case MOVING_FORWARD:
      if (isObstacleAhead()) {
        currentState = OBSTACLE_DETECTED;
        stopMotors();
        Serial.println("OBSTACLE! STOPPED");
      } else {
        moveForwardWithWallFollowing();
        Serial.println("MOVING - Wall Following");
      }
      break;
      
    case OBSTACLE_DETECTED:
      Serial.println("STOPPED - OBSTACLE AHEAD");
      stopMotors();
      
      // Optional: Check if obstacle cleared
      if (ranges.front_cm > OBSTACLE_DISTANCE + 5) {
        Serial.println("Path clear, resuming...");
        currentState = MOVING_FORWARD;
      }
      break;
      
    case STOPPED:
      Serial.println("STOPPED");
      stopMotors();
      break;
  }

  leftMotor.update();
  rightMotor.update();

  delay(50);
}
