#include "MotorPID.h"

// Initialize static pointer array
MotorPID *MotorPID::motorInstances[2] = {nullptr, nullptr};

// Constructor
MotorPID::MotorPID(int in1, int in2, int encA, int encB, bool inverted)
    : in1Pin(in1), in2Pin(in2), encAPin(encA), encBPin(encB), inverted(inverted),
      encoderCount(0), Kp(0.2), Ki(0.01), Kd(0.02), // Further reduced Kp, Ki, Kd values
      integral(0), lastError(0), lastTime(0) {}

void MotorPID::begin()
{
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(encAPin, INPUT_PULLUP);
    pinMode(encBPin, INPUT_PULLUP);
}

void MotorPID::attachEncoderISR()
{
    // You can assign instances manually if needed
    if (encAPin == 2)
    {
        motorInstances[0] = this;
        attachInterrupt(digitalPinToInterrupt(encAPin), handleEncoderA0, RISING);
    }
    else if (encAPin == 3)
    {
        motorInstances[1] = this;
        attachInterrupt(digitalPinToInterrupt(encAPin), handleEncoderA1, RISING);
    }
}

// Static ISR wrappers
void MotorPID::handleEncoderA0()
{
    if (motorInstances[0])
        motorInstances[0]->encoderISR();
}
void MotorPID::handleEncoderA1()
{
    if (motorInstances[1])
        motorInstances[1]->encoderISR();
}

void MotorPID::encoderISR()
{
    // Direction based on B phase
    if (digitalRead(encBPin) == HIGH)
        encoderCount += inverted ? -1 : 1;
    else
        encoderCount += inverted ? 1 : -1;
}

// Introduced a scaling factor to limit the maximum PWM output
const float PWM_SCALING_FACTOR = 0.5; // Scale down to 50% of the maximum speed

void MotorPID::updatePID(long targetPos)
{
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt < 0.01f)
        return;

    float error = (float)(targetPos - encoderCount);
    if (fabs(error) < 5)
    {
        stop();
        integral = 0;
        lastError = 0;
        lastTime = now;
        return;
    }

    integral += error * dt;
    integral = constrain(integral, -1000, 1000);
    float deriv = (error - lastError) / dt;
    float out = Kp * error + Ki * integral + Kd * deriv;
    out = constrain(out, -255, 255);

    // Apply scaling factor to reduce speed
    out *= PWM_SCALING_FACTOR;

    // Apply output to motor pins
    if (out > 0)
    {
        if (inverted)
        {
            analogWrite(in1Pin, 0);
            analogWrite(in2Pin, abs(out));
        }
        else
        {
            analogWrite(in1Pin, abs(out));
            analogWrite(in2Pin, 0);
        }
    }
    else
    {
        if (inverted)
        {
            analogWrite(in1Pin, abs(out));
            analogWrite(in2Pin, 0);
        }
        else
        {
            analogWrite(in1Pin, 0);
            analogWrite(in2Pin, abs(out));
        }
    }

    lastError = error;
    lastTime = now;
}

void MotorPID::stop()
{
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
}

long MotorPID::getCount()
{
    return encoderCount;
}
