#include <Arduino.h>

//Ultrasonic Sensor Pins
int TRIG_PIN_REAL = 2;  
int ECHO_PIN_REAL = 3;

float UltrasonicDistance(int TRIG_PIN, int ECHO_PIN);


void setup() {
    pinMode(TRIG_PIN_REAL, OUTPUT);
    pinMode(ECHO_PIN_REAL, INPUT);
}

void loop() {

 
}

// Function to measure distance using an ultrasonic sensor
float UltrasonicDistance(int TRIG_PIN, int ECHO_PIN) {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;
    return distance;
}
