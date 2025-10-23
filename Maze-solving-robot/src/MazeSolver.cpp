#include "MazeSolver.h"

MazeSolver::MazeSolver(MotorPID& left, MotorPID& right)
    : leftMotor(left), rightMotor(right), currentState(STOPPED), 
      irPins(nullptr), irPinCount(0), lastError(0.0f), lastUpdateTime(0) {
}

void MazeSolver::begin() {
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    
    currentState = MOVING_FORWARD;
}

void MazeSolver::setIRPins(const int* pins, int count) {
    irPins = pins;
    irPinCount = count;
}

float MazeSolver::ultrasonic_sensor_distance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 10000UL); // 10ms timeout
    if (duration == 0) {
        return -1.0;
    }
    float distance_cm = duration * 0.0343 / 2.0;
    return distance_cm;
}

void MazeSolver::readSensors() {
    ranges.front_cm = ultrasonic_sensor_distance(US_FRONT_TRIG, US_FRONT_ECHO);
    delay(10);
    ranges.left_cm = ultrasonic_sensor_distance(US_LEFT_TRIG, US_LEFT_ECHO);
    delay(10);
    ranges.right_cm = ultrasonic_sensor_distance(US_RIGHT_TRIG, US_RIGHT_ECHO);
    delay(10);
}

bool MazeSolver::allWhiteDetected() {
    if (irPins == nullptr || irPinCount == 0) {
        return false;  // IR pins not set, can't detect
    }
    
    int sumVal = 0;
    
    for (int i = 0; i < irPinCount; i++) {
        int value = digitalRead(irPins[i]);
        sumVal += value;
    }
    
    // If all sensors read LOW (0), all white detected
    Serial.print("IR Sum: ");
    Serial.println(sumVal);
    return (sumVal <= 2);
    
}

bool MazeSolver::sideIsOpen(float d) { 
    return d >= SIDE_OPEN_CM && d > 0; 
}

long MazeSolver::encLeft() { 
    return leftMotor.getEncoderCount(); 
}

long MazeSolver::encRight() { 
    return rightMotor.getEncoderCount(); 
}

void MazeSolver::encZeroBoth() {
    leftMotor.resetEncoder();   
    rightMotor.resetEncoder();  
}

void MazeSolver::calculateWallFollowingSpeeds(int &leftSpeed, int &rightSpeed) {
    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;

    if (dt <= 0) dt = 0.01f;
    lastUpdateTime = now;

    float error = ranges.left_cm - ranges.right_cm;

    if (fabsf(error) < ALIGNMENT_THRESHOLD) error = 0.0f;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = (Kp * 1.5 * error) + (Kd * derivative);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

    leftSpeed  = constrain(BASE_SPEED - correction, 0, 255);
    rightSpeed = constrain(BASE_SPEED + correction, 0, 255);
}

void MazeSolver::moveForwardWithWallFollowing() {
    int leftSpeed, rightSpeed;
    calculateWallFollowingSpeeds(leftSpeed, rightSpeed);
    
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

void MazeSolver::moveForward() {
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);
}

void MazeSolver::stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void MazeSolver::brakeShort() {
    stopMotors();
    delay(500);
}

void MazeSolver::forwardForMs(int pwmBase, long targetPulses) {
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);

    encZeroBoth();

    // Start with base speed; adjust inside loop using single-wall following
    leftMotor.setSpeed(pwmBase);
    rightMotor.setSpeed(pwmBase);

    while (true) {
        long cL = labs(encLeft());
        long cR = labs(encRight());
        long avg = (cL + cR) >> 1;

        if (avg >= targetPulses) break;  

        // Single-wall following: follow the side that is NOT open
        // readSensors();
        // const bool leftOpen  = sideIsOpen(ranges.left_cm);
        // const bool rightOpen = sideIsOpen(ranges.right_cm);

        // int lPWM = pwmBase;
        // int rPWM = pwmBase;

        // if (leftOpen ^ rightOpen) {
        //     Serial.println("Single-wall following active");
        //     const bool wallOnLeft = !leftOpen;  // single wall is the side that's not open
        //     const float wallDist  = wallOnLeft ? ranges.left_cm : ranges.right_cm;
        //     const float error     = TARGET_DIST_CM - wallDist; // +ve if too close
        //     float corr            = constrain(10 * error, -MAX_CORRECTION, MAX_CORRECTION);
        //     Serial.print("Correction: ");
        //     Serial.println(corr);

        //     if (wallOnLeft) {
        //         lPWM = constrain(pwmBase - (int)corr, 0, 255);
        //         rPWM = constrain(pwmBase + (int)corr, 0, 255);
        //     } else {
        //         lPWM = constrain(pwmBase + (int)corr, 0, 255);
        //         rPWM = constrain(pwmBase - (int)corr, 0, 255);
        //     }
        // }

        // leftMotor.setSpeed(lPWM);
        // rightMotor.setSpeed(rPWM);
        if (ultrasonic_sensor_distance(US_FRONT_TRIG, US_FRONT_ECHO) < 3 ) {
            // Serial.println("Obstacle detected! Stopping.");
            break;
        }
        // delayMicroseconds(700);
    }
    stopMotors();
    encZeroBoth();  
}

void MazeSolver::correctionRotate(){
    if (ranges.left_cm<3 && ranges.right_cm>10){
        leftMotor.setDirection(true);
        rightMotor.setDirection(false);
        leftMotor.setSpeed(50);
        rightMotor.setSpeed(50);
        delay(100);
        stopMotors();
    }
    if(ranges.right_cm<3 && ranges.left_cm>10){
        leftMotor.setDirection(false);
        rightMotor.setDirection(true);
        leftMotor.setSpeed(50);
        rightMotor.setSpeed(50);
        delay(100);
        stopMotors();
    }
}

void MazeSolver::Turn90(int dir) {
    const long target = COUNTS_PER_90; 
    const bool turnCW  = (dir > 0);
    const bool turnCCW = (dir < 0);

    leftMotor.setDirection(turnCCW); 
    rightMotor.setDirection(turnCW); 

    encZeroBoth();

    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);

    while (true) {
        long cL = labs(encLeft());
        long cR = labs(encRight());

        long avg = (cL + cR) >> 1;
        if (avg >= target) break;  

        delayMicroseconds(800);
    }

    stopMotors();
    encZeroBoth(); 
}

void MazeSolver::rotateLeft90() { 
    Turn90(+1); 
}

void MazeSolver::rotateRight90() { 
    Turn90(-1); 
}

void MazeSolver::reverseMotors(int duration_ms) {
    leftMotor.setDirection(false);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);
    delay(duration_ms);
    stopMotors();
}

void MazeSolver::rotateUTurn() {
    if (ranges.right_cm > ranges.left_cm) {
        rotateRight90();
        delay(100);
        // reverseMotors(200);
        rotateRight90();
        return;
    } else {
        rotateLeft90();
        delay(100);
        // reverseMotors(200);
        rotateLeft90();
        return;
    }
}

JunctionType MazeSolver::classifyJunction(const RangeReadings& r) {
    const bool frontBlocked = (r.front_cm > 0 && r.front_cm < 6);
    const bool leftOpen     = sideIsOpen(r.left_cm);
    const bool rightOpen    = sideIsOpen(r.right_cm);

    if (leftOpen && !rightOpen)  return JT_L_LEFT;
    else if (!leftOpen && rightOpen)  return JT_L_RIGHT;
    else if (leftOpen && rightOpen)   return frontBlocked ? JT_T : JT_CROSS_OR_CORNER;
    else                             return frontBlocked ? JT_DEAD_END : JT_STRAIGHT;
}

Decision MazeSolver::decideAction(JunctionType jt) {
    switch (jt) {
        case JT_T:                return DEC_LEFT;
        case JT_L_LEFT:           return DEC_LEFT;
        case JT_CROSS_OR_CORNER:  return DEC_LEFT;
        case JT_STRAIGHT:         return DEC_STRAIGHT;
        case JT_L_RIGHT:          return DEC_RIGHT;
        case JT_DEAD_END:         return DEC_UTURN;
        default:                  return DEC_NONE;
    }
}

void MazeSolver::executeDecision(Decision d) {
    switch (d) {
        case DEC_LEFT:      forwardForMs(BASE_SPEED, 330); rotateLeft90();  forwardForMs(BASE_SPEED, 200); break;
        case DEC_STRAIGHT:  break;
        case DEC_RIGHT:     forwardForMs(BASE_SPEED, 330); rotateRight90();  forwardForMs(BASE_SPEED, 200); break;
        case DEC_UTURN:     rotateUTurn();     break;
        default:            break;
    }
}

bool MazeSolver::update() {
    readSensors();

    Serial.print("Front: ");
    Serial.print(ranges.front_cm);
    Serial.print(" cm, Left: ");
    Serial.print(ranges.left_cm);
    Serial.print(" cm, Right: ");
    Serial.print(ranges.right_cm);
    Serial.println(" cm");

    switch (currentState) {
        case MOVING_FORWARD: {
            JunctionType jt = classifyJunction(ranges);
            Serial.println(jt);

            const bool mustDecide =
            (jt == JT_T) || (jt == JT_L_LEFT) || (jt == JT_L_RIGHT) ||
            (jt == JT_DEAD_END) || (ranges.front_cm > 0 && ranges.front_cm < FRONT_STOP_CM);

            if (mustDecide) {
                stopMotors();

                Decision d = decideAction(jt);
                Serial.println(d);

                executeDecision(d);
                currentState = MOVING_FORWARD;
                break;
            }
            moveForwardWithWallFollowing();
            break;
        }
        case STOPPED:
            stopMotors();
            break;
    }

    leftMotor.update();
    rightMotor.update();
    
    // Check if all IR sensors detect white and return the result
    return allWhiteDetected();
}
