#include <Arduino.h>

// const int s1 = 42;
const int s2 = 43;
const int s3 = 44;
const int s4 = 45;
const int s5 = 46;
const int s6 = 47;
const int s7 = 48;
const int s8 = 49;

const int IR = 10; 
// const int IROn = 36; 

void setup() {
  Serial.begin(9600);

  // pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);

  pinMode(IR, OUTPUT);
  // pinMode(IROn, OUTPUT);

  digitalWrite(IR, HIGH);   
  // digitalWrite(IROn, HIGH); 

  delay(1000);
}

void loop() {
  // int v1 = digitalRead(s1);
  int v2 = digitalRead(s2);
  int v3 = digitalRead(s3);
  int v4 = digitalRead(s4);
  int v5 = digitalRead(s5);
  int v6 = digitalRead(s6);
  int v7 = digitalRead(s7);
  int v8 = digitalRead(s8);

  Serial.print("IR Sensors: ");
  // Serial.print(v1); Serial.print(" ");
  Serial.print(v2); Serial.print(" ");
  Serial.print(v3); Serial.print(" ");
  Serial.print(v4); Serial.print(" ");
  Serial.print(v5); Serial.print(" ");
  Serial.print(v6); Serial.print(" ");
  Serial.print(v7); Serial.print(" ");
  Serial.println(v8);

  delay(200);
}
