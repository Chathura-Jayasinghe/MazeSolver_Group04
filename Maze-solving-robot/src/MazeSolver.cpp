#include "MazeSolver.h"

MazeSolver::MazeSolver(MotorPID &left, MotorPID &right)
    : leftMotor(left), rightMotor(right)
{

    // Initialize Robot at (0,0) facing North
    currX = 0;
    currY = 0;
    currDir = NORTH;

    // Initialize Map: No walls (0)
    for (int x = 0; x <= MAZE_SIZE; x++)
    {
        for (int y = 0; y <= MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            visited[x][y] = 0;  // Not visited yet
        }
    }
    
    // Mark starting position with its coordinates (0,0 = 00)
    visited[0][0] = (0 * 10) + 0;
}

void MazeSolver::begin()
{
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    // Configure IR sensors same as line follower
    const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(irPins[i], INPUT);
    }

    stopMotors();
}

void MazeSolver::runStep()
{
    updateWalls();

    Serial.println("Full maze:");
    for (int x = 0; x <= MAZE_SIZE; x++) {
        String row = "";
        for (int y = 0; y <= MAZE_SIZE; y++) {
            row += String(walls[x][y], BIN) + " ";
        }
        Serial.println("row: " + row);
    }

    Serial.println("[Current Position] X: " + String(currX) + " Y: " + String(currY) + " Dir: " + String(currDir));
    Serial.println("[Target Position] X: " + String(TARGET_X) + " Y: " + String(TARGET_Y));

    if (currX == TARGET_X && currY == TARGET_Y)
    {
        Serial.println("\n>>> TARGET REACHED! <<<\n");
        stopMotors();
        return; // DONE!
    }

    // 3. Decide Best Move
    Direction nextDir = getBestDirection();
    Serial.println("Next Direction: " + String(nextDir) + " from Current Direction: " + String(currDir));
    Serial.println("==============================================================\n");

    // 4. Execute Move
    delay(1000);
    turnTo(nextDir);
    moveOneCell();

    // 5. Update Virtual Coordinates
    if (currDir == NORTH)
        currY++;
    else if (currDir == EAST)
        currX++;
    else if (currDir == SOUTH)
        currY--;
    else if (currDir == WEST)
        currX--;
    
    // Store position coordinates (e.g., X=5, Y=6 -> 56)
    visited[currX][currY] = (currX * 10) + currY;

    stopMotors();
    delay(1000);
}

bool MazeSolver::isFinished()
{
    // Finish only when robot reaches target coordinates
    if (currX == TARGET_X && currY == TARGET_Y)
    {
        return true;
    }
    return false;
}

bool MazeSolver::isTargetDetectedIR()
{
    // Treat HIGH on any IR sensor as white target detected
    const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (digitalRead(irPins[i]) == HIGH)
        {
            return true;
        }
    }
    return false;
}

// --- Mapping & Sensors ---
void MazeSolver::updateWalls()
{
    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    bool wallFront = (f > 0 && f < 12);
    bool wallLeft = (l > 0 && l < 12);
    bool wallRight = (r > 0 && r < 12);

    // Map Relative (Front/Left/Right) to Absolute (N/E/S/W)
    if (wallFront)
    {
        if (currDir == NORTH)
            walls[currX][currY] |= WALL_NORTH;
        if (currDir == EAST)
            walls[currX][currY] |= WALL_EAST;
        if (currDir == SOUTH)
            walls[currX][currY] |= WALL_SOUTH;
        if (currDir == WEST)
            walls[currX][currY] |= WALL_WEST;
    }
    if (wallRight)
    {
        if (currDir == NORTH)
            walls[currX][currY] |= WALL_EAST;
        if (currDir == EAST)
            walls[currX][currY] |= WALL_SOUTH;
        if (currDir == SOUTH)
            walls[currX][currY] |= WALL_WEST;
        if (currDir == WEST)
            walls[currX][currY] |= WALL_NORTH;
    }
    if (wallLeft)
    {
        if (currDir == NORTH)
            walls[currX][currY] |= WALL_WEST;
        if (currDir == EAST)
            walls[currX][currY] |= WALL_NORTH;
        if (currDir == SOUTH)
            walls[currX][currY] |= WALL_EAST;
        if (currDir == WEST)
            walls[currX][currY] |= WALL_SOUTH;
    }

    // // Sync neighbors (If I have North wall, neighbor above has South wall)
    // if ((walls[currX][currY] & WALL_NORTH) && currY <= MAZE_SIZE - 1)
    //     walls[currX][currY + 1] |= WALL_SOUTH;
    // if ((walls[currX][currY] & WALL_EAST) && currX <= MAZE_SIZE - 1)
    //     walls[currX + 1][currY] |= WALL_WEST;
    // if ((walls[currX][currY] & WALL_SOUTH) && currY > 0)
    //     walls[currX][currY - 1] |= WALL_NORTH;
    // if ((walls[currX][currY] & WALL_WEST) && currX > 0)
    //     walls[currX - 1][currY] |= WALL_EAST;

    Serial.println("Full maze:");
    for (int x = 0; x <= MAZE_SIZE; x++) {
        String row = "";
        for (int y = 0; y <= MAZE_SIZE; y++) {
            row += String(walls[x][y], BIN) + " ";
        }
        Serial.println("row: " + row);
    }
}

Direction MazeSolver::getBestDirection()
{
    Direction bestDir = currDir;

    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    // Define distance thresholds
    const float MIN_FORWARD_DISTANCE = 15.0; // Minimum safe distance to move forward

    bool wallFront = (f > -2 && f < MIN_FORWARD_DISTANCE);
    bool wallLeft = (l > 0 && l < 12);
    bool wallRight = (r > 0 && r < 12);

    // Bit 2: Front wall, Bit 1: Left wall, Bit 0: Right wall
    int wallConfig = (wallFront << 2) | (wallLeft << 1) | wallRight;

    switch (wallConfig) {
        
        case 0b110:  
        bestDir = (Direction)((currDir + 1) % 4);
        Serial.println("Decision: FORCED RIGHT (front & left blocked)");
        break;
        
        case 0b101: 
        bestDir = (Direction)((currDir + 3) % 4);
        Serial.println("Decision: FORCED LEFT (front & right blocked)");
        break;
        
        case 0b011:  
        bestDir = currDir;
        Serial.println("Decision: CORRIDOR → Straight");
        break;
        
        case 0b001:  
        bestDir = (Direction)((currDir + 3) % 4);  
        Serial.println("Decision: L-T-JUNCTION () → turn left");
        break;
        
        case 0b010: 
        bestDir = (Direction)((currDir + 1) % 4);
        Serial.println("Decision: L-JUNCTION () → right ");
        break;
        
        case 0b111: 
            bestDir = (Direction)((currDir + 2) % 4);
            Serial.println("Decision: DEAD END → U-Turn");
            break;
            
        case 0b000:  
            bestDir = (Direction)((currDir + 3) % 4); 
            Serial.println("Decision: CROSSROADS → Left (left-hand rule)");
            break;

        case 0b100: 
            bestDir = (Direction)((currDir + 3) % 4); 
            Serial.println("Decision: T-JUNCTION → Left (left-hand rule)");
            break;

        // default:  // Should never reach here, but safety fallback
        //     if (!wallLeft) {
        //         bestDir = (Direction)((currDir + 3) % 4);
        //     } else if (!wallRight) {
        //         bestDir = (Direction)((currDir + 1) % 4);
        //     } else {
        //         bestDir = (Direction)((currDir + 2) % 4);
        //     }
        //     break;
    }
    return bestDir;
}

void MazeSolver::reset()
{
    // Reset robot pose and map
    currX = 0;
    currY = 0;
    currDir = NORTH;
    for (int x = 0; x <= MAZE_SIZE; x++)
    {
        for (int y = 0; y <= MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            visited[x][y] = 0;
        }
    }
    visited[0][0] = (0 * 10) + 0;  // Mark starting position
    
    Serial.println("System reset complete!");
    stopMotors();
}

void MazeSolver::turnTo(Direction targetDir)
{
    int diff = (targetDir - currDir);

    if (diff == 0)
    {
    }
    else if (diff == 1 || diff == -3)
    {
        turnRight();
    }
    else if (diff == -1 || diff == 3)
    {
        turnLeft();
    }
    else if (diff == 2 || diff == -2)
    {
        turnAround();
    }

    currDir = targetDir;
}

void MazeSolver::moveOneCell() {
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);

    float error = 0.0f;
    float lastError = 0.0f;
    float correction = 0.0f;

    float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    if (frontDist > 0 && frontDist < 5.0) {
        Serial.println("Breaking [Before]");
        return;
    }

    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);

    while (true)
    {
        long avg = (labs(encLeft()) + labs(encRight())) / 2;
        if (avg >= COUNTS_PER_CELL)
            break;

        float rightDist = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);
        delay(15);
        float leftDist = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
        delay(15);

        bool rightWallExists = (rightDist > 0 && rightDist < WALL_THRESHOLD * 1.2);
        bool leftWallExists = (leftDist > 0 && leftDist < WALL_THRESHOLD * 1.2);

        if (rightWallExists && leftWallExists)
        {
            error = leftDist - rightDist + 1;
        }
        else if (rightWallExists)
        {
            error = DESIRED_WALL_DISTANCE - rightDist;
        }
        else if (leftWallExists)
        {
            error = leftDist - DESIRED_WALL_DISTANCE + 1;
        }
        else
        {
            error = 0;
        }

        correction = KP * error + KD * (error - lastError);
        correction = constrain(correction, -30, 30);
        lastError = error;

        int leftSpeed = BASE_SPEED - correction;
        int rightSpeed = BASE_SPEED + correction;

        leftSpeed = constrain(leftSpeed, 40, 200);
        rightSpeed = constrain(rightSpeed, 40, 200);

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);

        // Safety: If too close to front wall, stop early
        float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if (frontDist > 0 && frontDist < 3.50) {
            Serial.println("Breaking [After]");
            break;
        }

        delay(10);
    }
    stopMotors();
}

void MazeSolver::turnLeft(){
    encZero();
    leftMotor.setDirection(false);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90);
    stopMotors();
}

void MazeSolver::turnRight()
{
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90);
    stopMotors();
}

void MazeSolver::turnAround(){
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_180)
        ;
    stopMotors();
}

void MazeSolver::stopMotors()
{
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

float MazeSolver::readSensor(int trig, int echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(5);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, 10000);
    if (duration == 0)
        return -1;
    return duration * 0.034 / 2;
}

long MazeSolver::encLeft() { return leftMotor.getEncoderCount(); }
long MazeSolver::encRight() { return rightMotor.getEncoderCount(); }
void MazeSolver::encZero()
{
    leftMotor.resetEncoder();
    rightMotor.resetEncoder();
}
