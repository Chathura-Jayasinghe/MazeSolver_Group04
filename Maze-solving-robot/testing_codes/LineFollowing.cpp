#include <Arduino.h>

IR Sensor Pins
#define IR1 43
#define IR2 44
#define IR3 45
#define IR4 46
#define IR5 47
#define IR6 48
#define IR7 49
#define IR8 50

    // // IR pins uno
    // #define IR1 2
    // #define IR2 3
    // #define IR3 4
    // #define IR4 5
    // #define IR5 6
    // #define IR6 7
    // #define IR7 8
    // #define IR8 9

    const int irPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
const int numSensors = 8;

Motor Pins mega
#define LEFT_MOTOR_IN1 9  // Left motor forward
#define LEFT_MOTOR_IN2 8  // Left motor reverse
#define RIGHT_MOTOR_IN1 6 // Right motor forward
#define RIGHT_MOTOR_IN2 5 // Right motor reverse

    // // test with uno
    // #define LEFT_IN 12  // Left motor direction (forward)
    // #define ENA 11      // Left motor enable (PWM)
    // #define RIGHT_IN 13 // Right motor direction (forward)
    // #define ENB 10      // Right motor enable (PWM)

    // line Following parameters
    int weights[numSensors] = {-3, -2, -1, 0, 0, 1, 2, 3}; // left to right weights
float Kp = 35.0;                                           // proportional gain kp 30 base 80 ta wadarara
float Kd = 0.0;                                            // derivative gain                      // kp =35 base 40 kd 0 turn okkoma wada
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

    // // Setup motor pins uno
    // pinMode(LEFT_IN, OUTPUT);
    // pinMode(RIGHT_IN, OUTPUT);
    // pinMode(ENA, OUTPUT);
    // pinMode(ENB, OUTPUT);

    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
}

void runMotors(int leftPWM, int rightPWM)
{
    // digitalWrite(LEFT_IN, leftPWM > 0 ? HIGH : LOW);
    // digitalWrite(RIGHT_IN, rightPWM > 0 ? HIGH : LOW);
    // analogWrite(ENA, abs(leftPWM));
    // analogWrite(ENB, abs(rightPWM));

    // // direction pins (HIGH = forward)
    // digitalWrite(LEFT_IN, HIGH);
    // digitalWrite(RIGHT_IN, HIGH);

    // PWM limits
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    nalogWrite(LEFT_MOTOR_IN1, leftPWM);
    analogWrite(LEFT_MOTOR_IN2, 0);

    analogWrite(RIGHT_MOTOR_IN1, rightPWM);
    analogWrite(RIGHT_MOTOR_IN2, 0);

    // analogWrite(ENA, leftPWM);
    // analogWrite(ENB, rightPWM);
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

    // Serial.println("test: forward");
    // runMotors(200, 200);
    // delay(2000);
    // Serial.println("test: stop");
    // runMotors(0, 0);
    // delay(2000);
    // Serial.println("test: turn right");
    // runMotors(0, 200);
    // delay(2000);
    // Serial.println("test: turn left");
    // runMotors(200, 0);
    // delay(2000);
    // runMotors(0, 0);
    // delay(2000);

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
