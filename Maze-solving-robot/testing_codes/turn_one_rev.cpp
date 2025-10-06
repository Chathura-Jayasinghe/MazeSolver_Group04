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

// Motor specifications
#define PPR 11
#define GEAR_RATIO 21
#define PULSES_PER_REV (PPR * GEAR_RATIO)

// PID Constants for Motor 1 (Left)
float Kp1 = 2.0;
float Ki1 = 0.1;
float Kd1 = 0.2;

// PID Constants for Motor 2 (Right)
float Kp2 = 2.0;
float Ki2 = 0.1;
float Kd2 = 0.2;

// Motor 1 variables
volatile long encoder1Count = 0;
long target1Position = 0;
float integral1 = 0;
float lastError1 = 0;
unsigned long lastTime1 = 0;

// Motor 2 variables
volatile long encoder2Count = 0;
long target2Position = 0;
float integral2 = 0;
float lastError2 = 0;
unsigned long lastTime2 = 0;

// Function prototypes
void encoder1ISR();
void encoder2ISR();
void pidControl1();
void pidControl2();
void stopMotor1();
void stopMotor2();
void stopAllMotors();

void setup() {
  Serial.begin(9600);
  
  // Motor 1 setup
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  
  // Motor 2 setup
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);
  
  Serial.println("=== Dual Motor PID Control Ready! ===");
  delay(2000);
  
  // Move both motors one revolution forward
  target1Position = PULSES_PER_REV;
  target2Position = PULSES_PER_REV;
  
  Serial.print("Target Motor 1: ");
  Serial.println(target1Position);
  Serial.print("Target Motor 2: ");
  Serial.println(target2Position);
}

void loop() {
  // Run both PID controllers
  pidControl1();
  pidControl2();
  
  // Print status every 200ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("M1: ");
    Serial.print(encoder1Count);
    Serial.print("/");
    Serial.print(target1Position);
    Serial.print(" | M2: ");
    Serial.print(encoder2Count);
    Serial.print("/");
    Serial.println(target2Position);
    lastPrint = millis();
  }
}

// ===== MOTOR 1 (LEFT) FUNCTIONS =====

void encoder1ISR() {
  if (digitalRead(ENCODER1_B) == HIGH) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void pidControl1() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime1) / 1000.0;
  
  if (deltaTime < 0.01) return; // Run at ~100Hz
  
  // Calculate error
  float error = target1Position - encoder1Count;
  
  // If close enough, stop
  if (abs(error) < 5) {
    stopMotor1();
    return;
  }
  
  // PID calculations
  integral1 += error * deltaTime;
  integral1 = constrain(integral1, -1000, 1000); // Anti-windup
  
  float derivative = (error - lastError1) / deltaTime;
  
  // PID output
  float output = (Kp1 * error) + (Ki1 * integral1) + (Kd1 * derivative);
  
  // Limit output
  output = constrain(output, -255, 255);
  
  // Apply to motor 1
  if (output > 0) {
    analogWrite(MOTOR1_IN1, abs(output));
    analogWrite(MOTOR1_IN2, 0);
  } else {
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, abs(output));
  }
  
  lastError1 = error;
  lastTime1 = currentTime;
}

void stopMotor1() {
  analogWrite(MOTOR1_IN1, 0);
  analogWrite(MOTOR1_IN2, 0);
}

// ===== MOTOR 2 (RIGHT) FUNCTIONS =====

void encoder2ISR() {
  // INVERTED: Swap increment/decrement to match motor direction
  if (digitalRead(ENCODER2_B) == HIGH) {
    encoder2Count--;  // Reversed
  } else {
    encoder2Count++;  // Reversed
  }
}

void pidControl2() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime2) / 1000.0;
  
  if (deltaTime < 0.01) return; // Run at ~100Hz
  
  // Calculate error
  float error = target2Position - encoder2Count;
  
  // If close enough, stop
  if (abs(error) < 5) {
    stopMotor2();
    return;
  }
  
  // PID calculations
  integral2 += error * deltaTime;
  integral2 = constrain(integral2, -1000, 1000); // Anti-windup
  
  float derivative = (error - lastError2) / deltaTime;
  
  // PID output
  float output = (Kp2 * error) + (Ki2 * integral2) + (Kd2 * derivative);
  
  // Limit output
  output = constrain(output, -255, 255);
  
  // INVERTED: Apply to motor 2 in reverse direction
  if (output > 0) {
    // Swapped IN1 and IN2 to reverse direction
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, abs(output));
  } else {
    analogWrite(MOTOR2_IN1, abs(output));
    analogWrite(MOTOR2_IN2, 0);
  }
  
  lastError2 = error;
  lastTime2 = currentTime;
}

void stopMotor2() {
  analogWrite(MOTOR2_IN1, 0);
  analogWrite(MOTOR2_IN2, 0);
}

// ===== UTILITY FUNCTIONS =====

void stopAllMotors() {
  stopMotor1();
  stopMotor2();
}
