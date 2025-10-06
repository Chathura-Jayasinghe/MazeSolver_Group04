#ifndef MOTORPID_H
#define MOTORPID_H

#include <Arduino.h>

class MotorPID
{
public:
    MotorPID(int in1, int in2, int encA, int encB, bool inverted = false);

    void begin();
    void updatePID(long targetPos);
    void stop();
    void attachEncoderISR();
    long getCount();

    static void handleEncoderA0(); // Static wrappers (for attachInterrupt)
    static void handleEncoderA1();

    // Static motor array references
    static MotorPID *motorInstances[2];

private:
    int in1Pin, in2Pin, encAPin, encBPin;
    bool inverted;
    volatile long encoderCount;
    float Kp, Ki, Kd;
    float integral, lastError;
    unsigned long lastTime;
    void encoderISR();
};

#endif
