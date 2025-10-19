#include <Arduino.h>

//  IR Sensor Pins
// #define IR1 43
// #define IR2 44
// #define IR3 45
// #define IR4 46
// #define IR5 47
// #define IR6 48
// #define IR7 49
// #define IR8 50

// IR pins uno
#define IR1 2
#define IR2 3
#define IR3 4
#define IR4 5
#define IR5 6
#define IR6 7
#define IR7 8
#define IR8 9

const int irPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
const int numSensors = 8;

//  Motor Pins mega
// #define LEFT_MOTOR_IN1 9  // Left motor forward
// #define LEFT_MOTOR_IN2 8  // Left motor reverse
// #define RIGHT_MOTOR_IN1 6 // Right motor forward
// #define RIGHT_MOTOR_IN2 5 // Right motor reverse

// test with uno
#define LEFT_IN 12  // Left motor direction (forward)
#define ENA 11      // Left motor enable (PWM)
#define RIGHT_IN 13 // Right motor direction (forward)
#define ENB 10      // Right motor enable (PWM)

void setup()
{
    Serial.begin(9600);
    // pinmode for IR sensor inputs
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(irPins[i], INPUT);
    }

    // Setup motor pins uno
    pinMode(LEFT_IN, OUTPUT);
    pinMode(RIGHT_IN, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void runMotors(int leftPWM, int rightPWM)
{
    digitalWrite(LEFT_IN, leftPWM > 0 ? HIGH : LOW);
    digitalWrite(RIGHT_IN, rightPWM > 0 ? HIGH : LOW);
    analogWrite(ENA, abs(leftPWM));
    analogWrite(ENB, abs(rightPWM));
}

//  Main Loop
void loop()
{
    // //// motor test code

    // digitalWrite(LEFT_IN, HIGH);
    // digitalWrite(RIGHT_IN, HIGH);
    // analogWrite(ENA, 200);
    // analogWrite(ENB, 200);

    // // check IR sensor

    // for (int i = 0; i < numSensors; i++)         //ir gives 1 for black, 0 for white
    // {                                                               // black line width 3-4 IR sensors
    //     int value = digitalRead(irPins[i]);
    //     Serial.print(value);
    //     Serial.print(" ");
    // }
    // Serial.println();
    // delay(500); //this will issue in later as it hold the microcontroller

    // check motor changing for turning

    Serial.println("test: forward");
    runMotors(200, 200);
    delay(2000);
    Serial.println("test: stop");
    runMotors(0, 0);
    delay(2000);
    Serial.println("test: turn right");
    runMotors(0, 200);
    delay(2000);
    Serial.println("test: turn left");
    runMotors(200, 0);
    delay(2000);
    runMotors(0, 0);
    delay(2000);
}
