#include <Arduino.h>

//  IR Sensor Pins 
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

//  Motor Pins 
#define LEFT_MOTOR_IN1 9    // Left motor forward
#define LEFT_MOTOR_IN2 8    // Left motor reverse
#define RIGHT_MOTOR_IN1 6   // Right motor forward
#define RIGHT_MOTOR_IN2 5   // Right motor reverse