#include <Arduino.h>

// === Pin Configuration ===
// Front sensor
#define FRONT_TRIG 22
#define FRONT_ECHO 24

// Left sensor (angled ~45°)
#define LEFT_TRIG 26
#define LEFT_ECHO 28

// Right sensor (angled ~45°)
#define RIGHT_TRIG 37
#define RIGHT_ECHO 36

// === Function to read distance ===
float readDistanceCM(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 25000UL); // 25ms timeout
    if (duration == 0)
    {
        return NAN; // No echo
    }
    float distance = duration / 58.0; // Convert to cm
    return distance;
}

void setup()
{
    Serial.begin(9600);

    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);

    Serial.println(F("=== Ultrasonic Sensor Test ==="));
    Serial.println(F("Front (32/33), Left (28/26), Right (36/37)"));
    Serial.println(F("--------------------------------------"));
}

void loop()
{
    float front = readDistanceCM(FRONT_TRIG, FRONT_ECHO);
    float left = readDistanceCM(LEFT_TRIG, LEFT_ECHO);
    float right = readDistanceCM(RIGHT_TRIG, RIGHT_ECHO);

    Serial.print(F("Front: "));
    if (isnan(front))
        Serial.print(F("No echo"));
    else
        Serial.print(front, 1);
    Serial.print(F(" cm | Left: "));
    if (isnan(left))
        Serial.print(F("No echo"));
    else
        Serial.print(left, 1);
    Serial.print(F(" cm | Right: "));
    if (isnan(right))
        Serial.print(F("No echo"));
    else
        Serial.print(right, 1);
    Serial.println(F(" cm"));

    delay(500); // half-second between reads
}
