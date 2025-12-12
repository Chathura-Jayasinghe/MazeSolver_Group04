#include <Arduino.h>
#include "MotorPID.h"
#include "MazeSolver.h"

// --- Hardware Definitions ---
#define MOTOR1_IN1 8
#define MOTOR1_IN2 9
#define ENCODER1_A 3
#define ENCODER1_B 7

#define MOTOR2_IN1 5
#define MOTOR2_IN2 6
#define ENCODER2_A 2
#define ENCODER2_B 4

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);

void setup() {
    Serial.begin(9600);
    
    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();

    Serial.println();
    Serial.println("FLOOD FILL MAZE SOLVER");
    Serial.println("Starting in 2 seconds...");
    delay(2000);
}

void loop() {
    // --- Mode selection (temporary booleans; replace with switch later) ---
    static bool modeScan = true;    // Mode 1: explore + build map until IR white
    // static bool modeFollow = false; // Mode 2: follow precomputed shortest path
    // static bool modeReset = false;  // Mode 3: clear memory and pose

    // Example toggling logic placeholder:
    // You can change these booleans at runtime based on a physical switch later.

    // if (modeReset) {
    //     mazeSolver.reset();
    //     modeReset = false;
    //     modeScan = true;
    //     modeFollow = false;
    //     Serial.println("RESET DONE");
    //     delay(500);
    //     return;
    // }

    if (modeScan) {
        if (true) {
            mazeSolver.runStep();
        } else {
            Serial.println("-------maze scanning completed-------");
            mazeSolver.computeShortestPath();
            // switch to follow mode
            // modeScan = false;
            // modeFollow = true;
            // Serial.println("PATH READY. Switching to follow mode.");
            delay(500);
        }
        return;
    }

    // if (modeFollow) {
    //     // Follow the stored shortest path one step at a time
    //     mazeSolver.followShortestPathStep();
    //     // When path done, stop
    //     if (mazeSolver.isFinished()) {
    //         Serial.println("TARGET REACHED - FOLLOW MODE COMPLETE");
    //         delay(1000);
    //     }
    //     return;
    // }
}



// #include <Arduino.h>

// // === Pin Configuration ===
// // Front sensor
// #define FRONT_TRIG 22
// #define FRONT_ECHO 24

// // Left sensor (angled ~45°)
// #define LEFT_TRIG 26
// #define LEFT_ECHO 28

// // Right sensor (angled ~45°)
// #define RIGHT_TRIG 37
// #define RIGHT_ECHO 36

// // === Function to read distance ===
// float readDistanceCM(int trigPin, int echoPin)
// {
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);

//     unsigned long duration = pulseIn(echoPin, HIGH, 25000UL); // 25ms timeout
//     if (duration == 0)
//     {
//         return NAN; // No echo
//     }
//     float distance = duration / 58.0; // Convert to cm
//     return distance;
// }

// void setup()
// {
//     Serial.begin(9600);

//     pinMode(FRONT_TRIG, OUTPUT);
//     pinMode(FRONT_ECHO, INPUT);
//     pinMode(LEFT_TRIG, OUTPUT);
//     pinMode(LEFT_ECHO, INPUT);
//     pinMode(RIGHT_TRIG, OUTPUT);
//     pinMode(RIGHT_ECHO, INPUT);

//     Serial.println(F("=== Ultrasonic Sensor Test ==="));
//     Serial.println(F("Front (32/33), Left (28/26), Right (36/37)"));
//     Serial.println(F("--------------------------------------"));
// }

// void loop()
// {
//     float front = readDistanceCM(FRONT_TRIG, FRONT_ECHO);
//     float left = readDistanceCM(LEFT_TRIG, LEFT_ECHO);
//     float right = readDistanceCM(RIGHT_TRIG, RIGHT_ECHO);

//     Serial.print(F("Front: "));
//     if (isnan(front))
//         Serial.print(F("No echo"));
//     else
//         Serial.print(front, 1);
//     Serial.print(F(" cm | Left: "));
//     if (isnan(left))
//         Serial.print(F("No echo"));
//     else
//         Serial.print(left, 1);
//     Serial.print(F(" cm | Right: "));
//     if (isnan(right))
//         Serial.print(F("No echo"));
//     else
//         Serial.print(right, 1);
//     Serial.println(F(" cm"));

//     delay(500); // half-second between reads
// }
