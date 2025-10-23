#include <Arduino.h>

//IR Sensor Pins
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

// Motor Pins
#define LEFT_MOTOR_IN1 9  // Left motor forward
#define LEFT_MOTOR_IN2 8  // Left motor reverse
#define RIGHT_MOTOR_IN1 6 // Right motor forward
#define RIGHT_MOTOR_IN2 5 // Right motor reverse

// line Following parameters
int weights[numSensors] = {-3, -2, -1, 0, 0, 1, 2, 3}; // left to right weights
float Kp = 35.0;                                       // proportional gain kp 30 base 80 - work
float Kd = 0.0;                                        // derivative gain                      // kp =35 base 40 kd 0 turn all turns work
int baseSpeed = 80;

float prevError = 0;
unsigned long prevTime = 0;

void setup()
{
    Serial.begin(9600);
    // pinmode for IR sensor inputs
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(irPins[i], INPUT);
    }
    
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
}

void runMotors(int leftPWM, int rightPWM)
{
    // PWM limits
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    analogWrite(LEFT_MOTOR_IN1, leftPWM);
    analogWrite(LEFT_MOTOR_IN2, 0);

    analogWrite(RIGHT_MOTOR_IN1, rightPWM);
    analogWrite(RIGHT_MOTOR_IN2, 0);
}

//  Main Loop
void loop()
{
    int sumVal = 0;
    int weightedSum = 0;

    // Read IR sensors
    for (int i = 0; i < numSensors; i++)
    {
        int value = digitalRead(irPins[i]); // 1 = black line
        if (value == HIGH)
        {
            weightedSum += weights[i];
            sumVal++;
        }
    }

    if (sumVal == 0)
    {
        // No line detected → stop or slow search
        runMotors(0, 100);
        Serial.println("Line lost!");
        delay(10); // previous value is 50
        return;
    }

    // compute proportional error
    float error = (float)weightedSum / sumVal;

    // derivative term
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    float derivative = 0;
    if (dt > 0.0)
        derivative = (error - prevError) / dt;

    // PD Controller Output
    float correction = Kp * error + Kd * derivative;

    // Adjust motor speeds
    int leftPWM = baseSpeed - correction;
    int rightPWM = baseSpeed + correction;

    // Run motors
    runMotors(leftPWM, rightPWM);

    // Debug output
    // Serial.print("Err: ");
    // Serial.print(error);
    // Serial.print(" | Corr: ");
    // Serial.print(correction);
    // Serial.print(" | L: ");
    // Serial.print(leftPWM);
    // Serial.print(" | R: ");
    // Serial.println(rightPWM);

    // prevError = error;
    // prevTime = now;
    // delay(20); // small delay for stability
}
