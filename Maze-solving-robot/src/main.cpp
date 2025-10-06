#include <Arduino.h>
#include "MotorPID.h"

#define REV_COUNT 20
#define PPR 11
#define GEAR_RATIO 21
#define PULSES_PER_REV (PPR * GEAR_RATIO)

MotorPID leftMotor(8, 9, 3, 7, false); // IN1, IN2, ENCA, ENCB, inverted?
MotorPID rightMotor(5, 6, 2, 4, true); // IN1, IN2, ENCA, ENCB, inverted

long targetLeft = REV_COUNT * PULSES_PER_REV;
long targetRight = REV_COUNT * PULSES_PER_REV;

void setup()
{
  Serial.begin(9600);
  leftMotor.begin();
  rightMotor.begin();

  leftMotor.attachEncoderISR();
  rightMotor.attachEncoderISR();

  Serial.println("=== Dual Motor PID (Modular) ===");
  Serial.print("Target L: ");
  Serial.println(targetLeft);
  Serial.print("Target R: ");
  Serial.println(targetRight);
}

void loop()
{
  leftMotor.updatePID(targetLeft);
  rightMotor.updatePID(targetRight);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 300)
  {
    lastPrint = millis();
    Serial.print("L: ");
    Serial.print(leftMotor.getCount());
    Serial.print("/");
    Serial.print(targetLeft);
    Serial.print(" | R: ");
    Serial.print(rightMotor.getCount());
    Serial.print("/");
    Serial.println(targetRight);
  }
}
