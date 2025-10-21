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

// Motor Pins (H-bridge style: IN1 = PWM forward, IN2 = PWM reverse)
#define LEFT_MOTOR_IN1 9   // Left motor forward (PWM)
#define LEFT_MOTOR_IN2 8   // Left motor reverse (PWM or digital)
#define RIGHT_MOTOR_IN1 6  // Right motor forward (PWM)
#define RIGHT_MOTOR_IN2 5  // Right motor reverse (PWM or digital)

// Line Following parameters
int   weights[numSensors] = {-3, -2, -1, 0, 0, 1, 2, 3}; // left→right
float Kp = 35.0;   // proportional gain
float Kd = 1.0;    // derivative gain (small, as requested)
int   baseSpeed = 80;

float prevError = 0.0f;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);

  // IR sensor inputs
  for (int i = 0; i < numSensors; i++) {
    pinMode(irPins[i], INPUT);
  }

  // Motor pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
}

void runMotors(int leftPWM, int rightPWM) {
  // Limit to [0, 255] (forward only)
  leftPWM  = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  analogWrite(LEFT_MOTOR_IN1, leftPWM);
  analogWrite(LEFT_MOTOR_IN2, 0);

  analogWrite(RIGHT_MOTOR_IN1, rightPWM);
  analogWrite(RIGHT_MOTOR_IN2, 0);
}

void loop() {
  int sumVal = 0;
  int weightedSum = 0;

  // Read IR sensors (assumes HIGH on line; flip if your sensors are active-LOW)
  for (int i = 0; i < numSensors; i++) {
    int value = digitalRead(irPins[i]); // 1 = black line
    if (value == HIGH) {
      weightedSum += weights[i];
      sumVal++;
    }
  }

  if (sumVal == 0) {
    // No line detected → gentle search / slow forward
    runMotors(0, 100);
    Serial.println("Line lost!");
    delay(10);
    return;
  }

  // Proportional error (centroid of active sensors)
  float error = (float)weightedSum / sumVal;

  // Derivative term
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = (error - prevError) / dt;
  }

  // PD controller output
  float correction = Kp * error + Kd * derivative;

  // Compute motor PWMs
  int leftPWM  = (int)(baseSpeed - correction);
  int rightPWM = (int)(baseSpeed + correction);

  runMotors(leftPWM, rightPWM);

  // Update history for KD
  prevError = error;
  prevTime  = now;

  // Small delay for stability
  delay(20);
}
