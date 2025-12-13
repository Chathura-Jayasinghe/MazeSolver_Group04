#include <Arduino.h>

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

// Global variables for pulse counting
volatile long motor1PulseCount = 0;
volatile long motor2PulseCount = 0;
volatile long motor1TargetPulses = 0;
volatile long motor2TargetPulses = 0;
volatile bool motor1Running = false;
volatile bool motor2Running = false;

// Motor control functions
void motor1Forward(int speed) {
  analogWrite(MOTOR1_IN1, speed);
  analogWrite(MOTOR1_IN2, 0);
}

void motor2Forward(int speed) {
  analogWrite(MOTOR2_IN1, speed);
  analogWrite(MOTOR2_IN2, 0);
}

void motor1Stop() {
  analogWrite(MOTOR1_IN1, 0);
  analogWrite(MOTOR1_IN2, 0);
  motor1Running = false;
}

void motor2Stop() {
  analogWrite(MOTOR2_IN1, 0);
  analogWrite(MOTOR2_IN2, 0);
  motor2Running = false;
}

// Interrupt service routines for encoders
void encoder1ISR() {
  motor1PulseCount++;
  if (motor1PulseCount >= motor1TargetPulses && motor1Running) {
    motor1Stop();
  }
}

void encoder2ISR() {
  motor2PulseCount++;
  if (motor2PulseCount >= motor2TargetPulses && motor2Running) {
    motor2Stop();
  }
}

// Function to move both motors for specified pulse count
void moveBothMotors(long pulses, int speed = 150) {
  // Reset counters
  motor1PulseCount = 0;
  motor2PulseCount = 0;
  motor1TargetPulses = pulses;
  motor2TargetPulses = pulses;
  
  // Start motors
  motor1Running = true;
  motor2Running = true;
  motor1Forward(speed);
  motor2Forward(speed);
  
  // Wait until both motors stop
  while (motor1Running || motor2Running) {
    delay(10);
  }
  
  Serial.print("Motor movement complete. Motor1: ");
  Serial.print(motor1PulseCount);
  Serial.print(" pulses, Motor2: ");
  Serial.print(motor2PulseCount);
  Serial.println(" pulses");
}

void setup() {
  Serial.begin(9600);
  
  // Initialize motor pins
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  
  // Initialize encoder pins
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);
  
  // Ensure motors are stopped initially
  motor1Stop();
  motor2Stop();
  
  Serial.println("Motor Pulse Control System Ready");
  Serial.println("Motors will move forward for specified pulse count");
}

void loop() {
  // Example: Move both motors for 100 pulses
  Serial.println("Moving motors for 100 pulses...");
  moveBothMotors(100);
  
  delay(3000); // Wait 3 seconds before next movement
  
  // Example: Move both motors for 200 pulses
  Serial.println("Moving motors for 200 pulses...");
  moveBothMotors(200);
  
  delay(3000); // Wait 3 seconds before next movement
}
