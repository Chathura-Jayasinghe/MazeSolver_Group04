/*
  MotorPID.h - Motor control with encoder feedback
  Simple motor control for maze navigation
*/

#ifndef MOTORPID_H
#define MOTORPID_H

#include <Arduino.h>

class MotorPID {
  public:
    // Constructor
    MotorPID(int in1Pin, int in2Pin, int encoderAPin, int encoderBPin, bool reversed = false);
    
    // Initialize motor and encoder
    void begin();
    
    // Set motor direction (true = forward, false = backward)
    void setDirection(bool forward);
    
    // Set motor speed (0-255)
    void setSpeed(int speed);
    
    // Stop the motor
    void stop();
    
    // Update motor (call in loop for encoder counting)
    void update();
    
    // Get current encoder count
    long getEncoderCount();
    
    // Reset encoder count
    void resetEncoder();
    
    // Get current speed setting
    int getSpeed();
    
    // Get current direction
    bool getDirection();
    
  private:
    // Motor pins
    int _in1Pin;
    int _in2Pin;
    
    // Encoder pins
    int _encoderAPin;
    int _encoderBPin;
    
    // Motor state
    int _speed;
    bool _forward;
    bool _reversed;  // For motors that need reversed polarity
    
    // Encoder count
    volatile long _encoderCount;
    
    // Apply speed and direction to motor
    void _applyMotorControl();
    
    // Static encoder interrupt handlers
    static void _encoderISR_A();
    static void _encoderISR_B();
    
    // Static instance pointers for ISR access
    static MotorPID* _instance1;
    static MotorPID* _instance2;
    int _instanceNum;
    
    // Instance encoder handler
    void _handleEncoder();
};

#endif