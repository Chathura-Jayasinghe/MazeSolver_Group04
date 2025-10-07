/*
  MotorPID.cpp - Motor control with encoder feedback
  Implementation file
*/

#include "MotorPID.h"

MotorPID* MotorPID::_instance1 = nullptr;
MotorPID* MotorPID::_instance2 = nullptr;

// Constructor
MotorPID::MotorPID(int in1Pin, int in2Pin, int encoderAPin, int encoderBPin, bool reversed) {
  _in1Pin = in1Pin;
  _in2Pin = in2Pin;
  _encoderAPin = encoderAPin;
  _encoderBPin = encoderBPin;
  _reversed = reversed;
  
  _speed = 0;
  _forward = true;
  _encoderCount = 0;
  
  // Assign instance number for ISR routing
  if (_instance1 == nullptr) {
    _instance1 = this;
    _instanceNum = 1;
  } else if (_instance2 == nullptr) {
    _instance2 = this;
    _instanceNum = 2;
  }
}

// Initialize motor and encoder
void MotorPID::begin() {
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);
  
  pinMode(_encoderAPin, INPUT_PULLUP);
  pinMode(_encoderBPin, INPUT_PULLUP);
  
  if (_instanceNum == 1) {
    attachInterrupt(digitalPinToInterrupt(_encoderAPin), _encoderISR_A, CHANGE);
  } else if (_instanceNum == 2) {
    attachInterrupt(digitalPinToInterrupt(_encoderAPin), _encoderISR_B, CHANGE);
  }
  
  stop();
}

// Set motor direction
void MotorPID::setDirection(bool forward) {
  _forward = forward;
  _applyMotorControl();
}

// Set motor speed (0-255)
void MotorPID::setSpeed(int speed) {
  // Constrain speed to valid range
  _speed = constrain(speed, 0, 255);
  _applyMotorControl();
}

// Stop the motor
void MotorPID::stop() {
  _speed = 0;
  digitalWrite(_in1Pin, LOW);
  digitalWrite(_in2Pin, LOW);
}

// Update motor (call in loop)
void MotorPID::update() {
  // This function can be used for PID control in future
  // For now, it just maintains current speed/direction
  _applyMotorControl();
}

// Get encoder count
long MotorPID::getEncoderCount() {
  return _encoderCount;
}

// Reset encoder count
void MotorPID::resetEncoder() {
  noInterrupts();
  _encoderCount = 0;
  interrupts();
}

// Get current speed
int MotorPID::getSpeed() {
  return _speed;
}

// Get current direction
bool MotorPID::getDirection() {
  return _forward;
}

// Apply motor control based on speed and direction
void MotorPID::_applyMotorControl() {
  bool actualForward = _reversed ? !_forward : _forward;
  
  if (_speed == 0) {
    // Stop motor
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
  } else {
    if (actualForward) {
      // Forward direction
      analogWrite(_in1Pin, _speed);
      digitalWrite(_in2Pin, LOW);
    } else {
      // Backward direction
      digitalWrite(_in1Pin, LOW);
      analogWrite(_in2Pin, _speed);
    }
  }
}

// Handle encoder pulse
void MotorPID::_handleEncoder() {
  // Read both encoder pins
  int a = digitalRead(_encoderAPin);
  int b = digitalRead(_encoderBPin);
  
  // Determine direction and increment/decrement count
  if (a == b) {
    _encoderCount++;
  } else {
    _encoderCount--;
  }
}

// Static ISR for motor 1 (encoder A pin)
void MotorPID::_encoderISR_A() {
  if (_instance1 != nullptr) {
    _instance1->_handleEncoder();
  }
}

// Static ISR for motor 2 (encoder A pin)
void MotorPID::_encoderISR_B() {
  if (_instance2 != nullptr) {
    _instance2->_handleEncoder();
  }
}