#include <Arduino.h>

// ===== IR Analog Pins (A8..A15) =====
#define IR1 A8
#define IR2 A9
#define IR3 A10
#define IR4 A11
#define IR5 A12
#define IR6 A13
#define IR7 A14
#define IR8 A15

const int numSensors = 8;
const int irPins[numSensors] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};

// ===== Analog detection tuning =====
const int IR_THRESHOLD = 900;      // tune (black line gives higher value in this logic)
const int MAX_STRENGTH = 400;      // clamp strength

// Motor Pins (forward-only wiring)
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 8
#define RIGHT_MOTOR_IN1 6
#define RIGHT_MOTOR_IN2 5

// Line follow params
int weights[numSensors] = {-3, -2, -1, 0, 0, 1, 2, 3};
float Kp = 35.0;
float Kd = 0.0;
int baseSpeed = 50;

float prevError = 0;
unsigned long prevTime = 0;

// ===== Search logic params (same idea as your class) =====
enum TurnState { NORMAL_FOLLOWING, SEARCHING_LEFT, SEARCHING_RIGHT };
TurnState turnState = NORMAL_FOLLOWING;

const unsigned long SEARCH_DURATION_MS = 700;  // increase to widen search range
const int TURN_SPEED_LINE = 90;                // turn speed during search
const int LINE_DETECT_SUM_MIN = 25;            // require some strength to accept "found"

// ---------- Motor helpers ----------
void runMotors(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  analogWrite(LEFT_MOTOR_IN1, leftPWM);
  analogWrite(LEFT_MOTOR_IN2, 0);

  analogWrite(RIGHT_MOTOR_IN1, rightPWM);
  analogWrite(RIGHT_MOTOR_IN2, 0);
}

// forward-only "turn": stop one wheel, run the other
void turnLeft_line()  { runMotors(0, TURN_SPEED_LINE); }
void turnRight_line() { runMotors(TURN_SPEED_LINE, 0); }

void stopMotors() { runMotors(0, 0); }

// ---------- Analog sensor read helpers ----------
void readLineAnalog(long &weightedSum, long &sumStrength) {
  weightedSum = 0;
  sumStrength = 0;

  for (int i = 0; i < numSensors; i++) {
    int a = analogRead(irPins[i]);

    int strength = a - IR_THRESHOLD;   // if black gives higher readings
    if (strength < 0) strength = 0;
    if (strength > MAX_STRENGTH) strength = MAX_STRENGTH;

    weightedSum += (long)weights[i] * (long)strength;
    sumStrength += strength;
  }
}

bool lineDetectedAnalog() {
  long ws, ss;
  readLineAnalog(ws, ss);
  return (ss >= LINE_DETECT_SUM_MIN);
}

// Search for line by turning left OR right for SEARCH_DURATION_MS
bool searchForLineAnalog(bool searchLeft) {
  unsigned long searchStart = millis();

  while (millis() - searchStart < SEARCH_DURATION_MS) {
    if (searchLeft) turnLeft_line();
    else           turnRight_line();

    delay(10);

    if (lineDetectedAnalog()) {
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  prevTime = millis();
  turnState = NORMAL_FOLLOWING;
}

void loop() {
  long weightedSum = 0;
  long sumStrength = 0;

  // Read analog line position
  readLineAnalog(weightedSum, sumStrength);

  switch (turnState) {

    case NORMAL_FOLLOWING: {
      // If no line -> start LEFT search first (same logic as your earlier code)
      if (sumStrength < LINE_DETECT_SUM_MIN) {
        Serial.println("Line lost - starting LEFT search");
        turnState = SEARCHING_LEFT;
        return;
      }

      // PD follow using analog weighted average
      float error = (float)weightedSum / (float)sumStrength;

      unsigned long now = millis();
      float dt = (now - prevTime) / 1000.0f;
      float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;

      float correction = Kp * error + Kd * derivative;

      int leftPWM  = baseSpeed - (int)correction;
      int rightPWM = baseSpeed + (int)correction;

      runMotors(leftPWM, rightPWM);

      prevError = error;
      prevTime  = now;
      break;
    }

    case SEARCHING_LEFT: {
      Serial.println("Searching LEFT...");
      if (searchForLineAnalog(true)) {
        Serial.println("Found line on LEFT!");
        turnState = NORMAL_FOLLOWING;

        // reset timing so derivative doesn't spike after long search
        prevTime = millis();
        prevError = 0;
      } else {
        Serial.println("Not found on LEFT, trying RIGHT...");
        turnState = SEARCHING_RIGHT;
      }
      break;
    }

    case SEARCHING_RIGHT: {
      Serial.println("Searching RIGHT...");
      if (searchForLineAnalog(false)) {
        Serial.println("Found line on RIGHT!");
        turnState = NORMAL_FOLLOWING;

        prevTime = millis();
        prevError = 0;
      } else {
        Serial.println("Not found on either side -> STOP");
        stopMotors();
        delay(1000);
        turnState = NORMAL_FOLLOWING;
      }
      break;
    }
  }

  delay(20);
}
