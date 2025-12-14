#include "MazeSolver.h"

MazeSolver::MazeSolver(MotorPID &left, MotorPID &right)
    : leftMotor(left), rightMotor(right)
{
    currX = 0;
    currY = 0;
    currDir = NORTH;

    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            dist[x][y] = 255;
        }
    }

    pathLen = 0;
    pathIndex = 0;
}

void MazeSolver::begin()
{
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(irPins[i], INPUT);
    }

    stopMotors();
}

void MazeSolver::runStep()
{
    Direction nextDir = getBestDirection();
 
    turnTo(nextDir);
    moveOneCell();
    delay(1000);

    if (currDir == NORTH)
        currY++;
    else if (currDir == EAST)
        currX++;
    else if (currDir == SOUTH)
        currY--;
    else if (currDir == WEST)
        currX--;

    stopMotors();
    delay(500);
}

bool MazeSolver::isFinished()
{
    if (currX == TARGET_X && currY == TARGET_Y)
    {
        return true;
    }
    return isTargetDetectedIR();
}

bool MazeSolver::isTargetDetectedIR()
{
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

void MazeSolver::floodFill()
{
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            dist[x][y] = 255;
        }
    }

    struct Point
    {
        int x, y;
    };
    Point queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    dist[TARGET_X][TARGET_Y] = 0;
    queue[tail++] = {TARGET_X, TARGET_Y};

    while (head != tail)
    {
        Point p = queue[head++];
        int d = dist[p.x][p.y];

        if (p.y < MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_NORTH))
        {
            if (dist[p.x][p.y + 1] == 255)
            {
                dist[p.x][p.y + 1] = d + 1;
                queue[tail++] = {p.x, p.y + 1};
            }
        }
        if (p.x < MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_EAST))
        {
            if (dist[p.x + 1][p.y] == 255)
            {
                dist[p.x + 1][p.y] = d + 1;
                queue[tail++] = {p.x + 1, p.y};
            }
        }
        if (p.y > 0 && !(walls[p.x][p.y] & WALL_SOUTH))
        {
            if (dist[p.x][p.y - 1] == 255)
            {
                dist[p.x][p.y - 1] = d + 1;
                queue[tail++] = {p.x, p.y - 1};
            }
        }
        if (p.x > 0 && !(walls[p.x][p.y] & WALL_WEST))
        {
            if (dist[p.x - 1][p.y] == 255)
            {
                dist[p.x - 1][p.y] = d + 1;
                queue[tail++] = {p.x - 1, p.y};
            }
        }
    }
}

void MazeSolver::updateWalls()
{
    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    bool wallFront = (f > 0 && f < 10);
    bool wallLeft = (l > 0 && l < 10);
    bool wallRight = (r > 0 && r < 8);

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

    if ((walls[currX][currY] & WALL_NORTH) && currY < MAZE_SIZE - 1)
        walls[currX][currY + 1] |= WALL_SOUTH;
    if ((walls[currX][currY] & WALL_EAST) && currX < MAZE_SIZE - 1)
        walls[currX + 1][currY] |= WALL_WEST;
    if ((walls[currX][currY] & WALL_SOUTH) && currY > 0)
        walls[currX][currY - 1] |= WALL_NORTH;
    if ((walls[currX][currY] & WALL_WEST) && currX > 0)
        walls[currX - 1][currY] |= WALL_EAST;
}

Direction MazeSolver::getBestDirection()
{
       Direction bestDir = currDir;

    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    const float MIN_FORWARD_DISTANCE = 15.0;

    bool wallFront = (f > -2 && f < MIN_FORWARD_DISTANCE);
    bool wallLeft = (l > 0 && l < 12);
    bool wallRight = (r > 0 && r < 12);

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
    }
    return bestDir;
}

Direction MazeSolver::getBestDirectionAt(int x, int y)
{
    int minDist = 255;
    Direction bestDir = NORTH;

    if (y < MAZE_SIZE - 1 && !(walls[x][y] & WALL_NORTH))
    {
        if (dist[x][y + 1] < minDist)
        {
            minDist = dist[x][y + 1];
            bestDir = NORTH;
        }
    }
    if (x < MAZE_SIZE - 1 && !(walls[x][y] & WALL_EAST))
    {
        if (dist[x + 1][y] < minDist)
        {
            minDist = dist[x + 1][y];
            bestDir = EAST;
        }
    }
    if (y > 0 && !(walls[x][y] & WALL_SOUTH))
    {
        if (dist[x][y - 1] < minDist)
        {
            minDist = dist[x][y - 1];
            bestDir = SOUTH;
        }
    }
    if (x > 0 && !(walls[x][y] & WALL_WEST))
    {
        if (dist[x - 1][y] < minDist)
        {
            minDist = dist[x - 1][y];
            bestDir = WEST;
        }
    }
    return bestDir;
}

void MazeSolver::computeShortestPath()
{
    floodFill();
    int x = 0, y = 0;
    Direction d;
    pathLen = 0;
    pathIndex = 0;

    if (dist[x][y] == 255)
        return;

    for (int steps = 0; steps < MAZE_SIZE * MAZE_SIZE; steps++)
    {
        if (x == TARGET_X && y == TARGET_Y)
            break;
        d = getBestDirectionAt(x, y);
        path[pathLen++] = d;
        if (d == NORTH)
            y++;
        else if (d == EAST)
            x++;
        else if (d == SOUTH)
            y--;
        else if (d == WEST)
            x--;
    }
}

void MazeSolver::followShortestPathStep()
{
    if (pathIndex >= pathLen)
    {
        stopMotors();
        return;
    }
    Direction nextDir = path[pathIndex];
    turnTo(nextDir);
    moveOneCell();
    if (currDir == NORTH)
        currY++;
    else if (currDir == EAST)
        currX++;
    else if (currDir == SOUTH)
        currY--;
    else if (currDir == WEST)
        currX--;
    stopMotors();
    pathIndex++;
}

void MazeSolver::reset()
{
    currX = 0;
    currY = 0;
    currDir = NORTH;
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            dist[x][y] = 255;
        }
    }
    pathLen = 0;
    pathIndex = 0;
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
    else
    {
        turnAround();
    }

    currDir = targetDir;
}

void MazeSolver::moveOneCell()
{
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);

    float error = 0;
    float lastError = 0;
    float correction = 0;

    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);

    while (true)
    {
        long avg = (labs(encLeft()) + labs(encRight())) / 2;
        if (avg >= COUNTS_PER_CELL)
            break;

        float rightDist = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);
        float leftDist = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);

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
        lastError = error;

        int leftSpeed = BASE_SPEED - correction;
        int rightSpeed = BASE_SPEED + correction;

        leftSpeed = constrain(leftSpeed, 0, 200);
        rightSpeed = constrain(rightSpeed, 0, 200);

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);

        delay(10);
    }
    stopMotors();
}

void MazeSolver::turnLeft()
{
    encZero();
    leftMotor.setDirection(false);
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90)
        ;
    stopMotors();
}

void MazeSolver::turnRight()
{
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90)
        ;
    stopMotors();
}

void MazeSolver::turnAround()
{
    turnRight();
    delay(200);
    turnRight();
}

void MazeSolver::stopMotors()
{
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

float MazeSolver::readSensor(int trig, int echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
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