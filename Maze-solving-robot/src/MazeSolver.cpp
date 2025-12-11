#include "MazeSolver.h"

MazeSolver::MazeSolver(MotorPID &left, MotorPID &right)
    : leftMotor(left), rightMotor(right)
{

    // Initialize Robot at (0,0) facing North
    currX = 0;
    currY = 0;
    currDir = NORTH;

    // Initialize Map: No walls (0), Unknown distance (255)
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
    // 1. Read Sensors & Update Walls
    // updateWalls();

    // 2. Check if we reached target
    // if (currX == TARGET_X && currY == TARGET_Y)
    // {
    //     stopMotors();
    //     return; // DONE!
    // }

    // 3. Recalculate Distances (Flood Fill)
    // floodFill();

    // 4. Decide Best Move
    Direction nextDir = getBestDirection();

    Serial.println("====================================================================================");
    Serial.println("Current Position (" + String(currX) + "," + String(currY) + ")");
    Serial.println("current robot Direction - before turn: " + String(currDir));
    Serial.println("current cell wall " + String(walls[currX][currY], BIN));

    // 5. Execute Move
    turnTo(nextDir);
    moveOneCell();
    delay(1000);

    Serial.println("Before Turn Next Direction: " + String(nextDir));
    Serial.println("Move one cell");

    // 6. Update Virtual Coordinates
    if (currDir == NORTH)
        currY++;
    else if (currDir == EAST)
        currX++;
    else if (currDir == SOUTH)
        currY--;
    else if (currDir == WEST)
        currX--;

    Serial.println("Next Position after move i cell: (" + String(currX) + "," + String(currY) + ")");
    Serial.println("====================================================================================");
    Serial.println();

    stopMotors();
    delay(500);
}

bool MazeSolver::isFinished()
{
    // Finish if coordinates reach target or IR sees white target
    if (currX == TARGET_X && currY == TARGET_Y)
    {
        return true;
    }
    return isTargetDetectedIR();
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

// --- Flood Fill Implementation ---
void MazeSolver::floodFill()
{
    // Reset distances
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            dist[x][y] = 255;
        }
    }

    // Queue for BFS (Circular buffer)
    struct Point
    {
        int x, y;
    };
    Point queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    // Start at Target
    dist[TARGET_X][TARGET_Y] = 0;
    queue[tail++] = {TARGET_X, TARGET_Y};

    while (head != tail)
    {
        Point p = queue[head++];
        int d = dist[p.x][p.y];

        // Check North Neighbor (x, y+1)
        if (p.y < MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_NORTH))
        {
            if (dist[p.x][p.y + 1] == 255)
            {
                dist[p.x][p.y + 1] = d + 1;
                queue[tail++] = {p.x, p.y + 1};
            }
        }
        // Check East Neighbor (x+1, y)
        if (p.x < MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_EAST))
        {
            if (dist[p.x + 1][p.y] == 255)
            {
                dist[p.x + 1][p.y] = d + 1;
                queue[tail++] = {p.x + 1, p.y};
            }
        }
        // Check South Neighbor (x, y-1)
        if (p.y > 0 && !(walls[p.x][p.y] & WALL_SOUTH))
        {
            if (dist[p.x][p.y - 1] == 255)
            {
                dist[p.x][p.y - 1] = d + 1;
                queue[tail++] = {p.x, p.y - 1};
            }
        }
        // Check West Neighbor (x-1, y)
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

// --- Mapping & Sensors ---
void MazeSolver::updateWalls()
{
    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);
    Serial.println("Sensor Readings - Front: " + String(f) + " cm, Left: " + String(l) + " cm, Right: " + String(r) + " cm");

    bool wallFront = (f > 0 && f < 10);
    bool wallLeft = (l > 0 && l < 10);
    bool wallRight = (r > 0 && r < 8);

    Serial1.println("#################################################################################");
    Serial1.println();

    Serial.print("frontwall -  " + String(wallFront));
    Serial.print("/ leftwall -  " + String(wallLeft));
    Serial.println("/ rightwall -  " + String(wallRight));
    Serial1.println();

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

    // Sync neighbors (If I have North wall, neighbor above has South wall)
    if ((walls[currX][currY] & WALL_NORTH) && currY < MAZE_SIZE - 1)
        walls[currX][currY + 1] |= WALL_SOUTH;
    if ((walls[currX][currY] & WALL_EAST) && currX < MAZE_SIZE - 1)
        walls[currX + 1][currY] |= WALL_WEST;
    if ((walls[currX][currY] & WALL_SOUTH) && currY > 0)
        walls[currX][currY - 1] |= WALL_NORTH;
    if ((walls[currX][currY] & WALL_WEST) && currX > 0)
        walls[currX - 1][currY] |= WALL_EAST;

    Serial.println("Full maze:");
    for (int x = 0; x < MAZE_SIZE; x++) {
        String row = "";
        for (int y = 0; y < MAZE_SIZE; y++) {
            row += String(walls[x][y], BIN) + " ";
        }
        Serial.println("row: " + row);
    }
}

Direction MazeSolver::getBestDirection(){
    Direction bestDir = currDir;

    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    // Define distance thresholds
    const float MIN_FORWARD_DISTANCE = 15.0;  // Minimum safe distance to move forward
    
    bool wallFront = (f > -2 && f < MIN_FORWARD_DISTANCE);
    bool wallLeft = (l > 0 && l < WALL_THRESHOLD);
    bool wallRight = (r > 0 && r < WALL_THRESHOLD);

    Serial.print("Sensors: F=" + String(f) + " L=" + String(l) + " R=" + String(r));
    Serial.println(" | Walls: F=" + String(wallFront) + " L=" + String(wallLeft) + " R=" + String(wallRight));

    // Create a case value based on wall configuration (3-bit: FLR)
    // Bit 2: Front wall, Bit 1: Left wall, Bit 0: Right wall
    int wallConfig = (wallFront << 2) | (wallLeft << 1) | wallRight;

    switch (wallConfig) {
        case 0b111:  // All walls (dead end)
            // bestDir = (Direction)((currDir + 2) % 4);R4
            Serial.println("Decision: DEAD END → U-Turn");
            break;

        case 0b110:  // Front and left blocked, right open
            bestDir = (Direction)((currDir + 1) % 4);
            Serial.println("Decision: FORCED RIGHT (front & left blocked)");
            break;

        case 0b101:  // Front and right blocked, left open
            bestDir = (Direction)((currDir + 3) % 4);
            Serial.println("Decision: FORCED LEFT (front & right blocked)");
            break;

        case 0b011:  // Front clear, both sides blocked (corridor)
            bestDir = currDir;
            Serial.println("Decision: CORRIDOR → Straight");
            break;

        case 0b001:  // Front clear, left blocked, right open (L-junction)
            bestDir = (Direction)((currDir + 3) % 4);  // LEFT has priority!

            Serial.println("Decision: L-JUNCTION () → turn left");
            break;

        case 0b010:  // Front clear, left open, right blocked (L-junction)
            bestDir = (Direction)((currDir + 1) % 4);
            Serial.println("Decision: L-JUNCTION () → right ");
            break;

        case 0b000:  // All directions clear (open space or crossroads)
            // LEFT-HAND RULE PRIORITY: Left > Straight > Right
            bestDir = (Direction)((currDir + 3) % 4);  // Always turn left
            Serial.println("Decision: CROSSROADS → Left (left-hand rule)");
            break;

        case 0b100:  // Front blocked, both sides open (T-junction)
            bestDir = (Direction)((currDir + 3) % 4);  // Left-hand rule
            Serial.println("Decision: T-JUNCTION → Left (left-hand rule)");
            break;

        default:  // Should never reach here, but safety fallback
            if (!wallLeft) {
                bestDir = (Direction)((currDir + 3) % 4);
                Serial.println("Decision: FALLBACK → Left");
            } else if (!wallRight) {
                bestDir = (Direction)((currDir + 1) % 4);
                Serial.println("Decision: FALLBACK → Right");
            } else {
                bestDir = (Direction)((currDir + 2) % 4);
                Serial.println("Decision: FALLBACK → U-Turn");
            }
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
    // Recompute flood fill distances first
    floodFill();

    // Build path from start (0,0) to target using decreasing dist
    int x = 0, y = 0;
    Direction d;
    pathLen = 0;
    pathIndex = 0;

    // Guard: if start unreachable, keep empty path
    if (dist[x][y] == 255)
        return;

    // Limit steps to grid size to avoid infinite loops
    for (int steps = 0; steps < MAZE_SIZE * MAZE_SIZE; steps++)
    {
        if (x == TARGET_X && y == TARGET_Y)
            break;
        d = getBestDirectionAt(x, y);
        path[pathLen++] = d;
        // advance virtual position
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
    // Reset robot pose and map
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
        Serial.println("// Straight - Do nothing");
    }
    else if (diff == 1 || diff == -3)
    {
        Serial.println("!!!!!!turn right called");
        turnRight();
    }
    else if (diff == -1 || diff == 3)
    {
        Serial.println("!!!!!!turn left called");
        turnLeft();
    }
    else
    {
        turnAround();
    }

    currDir = targetDir;
}

void MazeSolver::moveOneCell(){
    Serial.println("-----------move one cell--------------");
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);

    float error = 0;
    float lastError = 0;
    float correction = 0;

     float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if (frontDist > 0 && frontDist < 5.0) {
            Serial.println("**************breaking...");
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

        // Safety: If too close to front wall, stop early
        float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if (frontDist > 0 && frontDist < 3.0) {
            Serial.println("**************breaking...");
            break;
        }

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

void MazeSolver::turnAround()
{
    // turnRight();
    // delay(200);
    // turnRight();
     encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED);
    rightMotor.setSpeed(TURN_SPEED);
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_180);
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