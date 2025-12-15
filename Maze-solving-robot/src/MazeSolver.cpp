#include "MazeSolver.h"
#include "LineFollower.h"

#define NUM_SENSORS 8

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
    updateWalls();
    floodFill();
    Direction nextDir = getBestDirection();

    turnTo(nextDir);
    moveOneCell();
    delay(10);

    if (currDir == NORTH && currY < MAZE_SIZE - 1)
        currY++;
    else if (currDir == EAST && currX < MAZE_SIZE - 1)
        currX++;
    else if (currDir == SOUTH && currY > 0)
        currY--;
    else if (currDir == WEST && currX > 0)
        currX--;

    stopMotors();
    delay(10);
    saveMazeToEEPROM();
}

void MazeSolver::runStepSmallMaze()
{
    Direction nextDir = getBestDirection();

    turnTo(nextDir);
    moveOneCell(COUNTS_PER_CELL_SMALL);
    delay(10);

    stopMotors();
 
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
    // Serial.println("Sensor Readings - Front: " + String(f) + " cm, Left: " + String(l) + " cm, Right: " + String(r) + " cm");

    bool wallFront = (f > 0 && f < 10);
    bool wallLeft = (l > 0 && l < 10);
    bool wallRight = (r > 0 && r < 8);

    // Serial1.println("#################################################################################");
    // Serial1.println();

    // Serial.print("frontwall -  " + String(wallFront));
    // Serial.print("/ leftwall -  " + String(wallLeft));
    // Serial.println("/ rightwall -  " + String(wallRight));
    // Serial1.println();

    // Add maze boundary walls
    if (currX == 0)
        walls[currX][currY] |= WALL_WEST;
    if (currX == MAZE_SIZE - 1)
        walls[currX][currY] |= WALL_EAST;
    if (currY == 0)
        walls[currX][currY] |= WALL_SOUTH;
    if (currY == MAZE_SIZE - 1)
        walls[currX][currY] |= WALL_NORTH;

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

    if (f < 0 && l < 0 && r < 0) {
        f = 100;
        l = 100;
        r = 100;
    }

    const float MIN_FORWARD_DISTANCE = 12.0;

    bool wallFront = (f > 0 && f < MIN_FORWARD_DISTANCE);
    bool wallLeft = (l > 0 && l < 12);
    bool wallRight = (r > 0 && r < 12);

    if (currDir == NORTH && currY >= MAZE_SIZE - 1) wallFront = true;
    if (currDir == EAST && currX >= MAZE_SIZE - 1) wallFront = true;
    if (currDir == SOUTH && currY <= 0) wallFront = true;
    if (currDir == WEST && currX <= 0) wallFront = true;

    int wallConfig = (wallFront << 2) | (wallLeft << 1) | wallRight;

    switch (wallConfig) {
        
        case 0b110:  
        bestDir = (Direction)((currDir + 1) % 4);
        break;
        
        case 0b101: 
        bestDir = (Direction)((currDir + 3) % 4);
        break;
        
        case 0b011:  
        bestDir = currDir;
        break;
        
        case 0b001:  
        bestDir = (Direction)((currDir + 3) % 4);  
        break;
        
        case 0b010: 
        bestDir = (Direction)((currDir + 1) % 4);
        break;
        
        case 0b111: 
            bestDir = (Direction)((currDir + 2) % 4);
            break;
            
        case 0b000:  
            bestDir = (Direction)((currDir + 3) % 4); 
            break;

        case 0b100: 
            bestDir = (Direction)((currDir + 3) % 4);
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

void MazeSolver::moveOneCell(int countsPerCell)
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

        // Safety: If too close to front wall, stop early
        float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if (frontDist > 0 && frontDist < 3) {
            // Serial.println("**************breaking...");
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

void MazeSolver::saveMazeToEEPROM()
{
    int addr = 0;
    
    EEPROM.update(addr++, currX);
    EEPROM.update(addr++, currY);
    EEPROM.update(addr++, (byte)currDir);
    
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            EEPROM.update(addr++, walls[x][y]);
        }
    }
    
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            EEPROM.update(addr++, dist[x][y]);
        }
    }
}

void MazeSolver::loadMazeFromEEPROM()
{
    int addr = 0;
    
    currX = EEPROM.read(addr++);
    currY = EEPROM.read(addr++);
    currDir = (Direction)EEPROM.read(addr++);
    
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            walls[x][y] = EEPROM.read(addr++);
        }
    }
    
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            dist[x][y] = EEPROM.read(addr++);
        }
    }
}

void MazeSolver::displayMazeTables()
{
    // Serial.println("\n========================================");
    // Serial.println("WALLS ARRAY (by coordinates)");
    // Serial.println("========================================");
    // Serial.print("   ");
    // for (int x = 0; x < MAZE_SIZE; x++) {
    //     Serial.print("  X" + String(x) + " ");
    // }
    // Serial.println();
    
    // for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    //     Serial.print("Y" + String(y) + " ");
    //     for (int x = 0; x < MAZE_SIZE; x++) {
    //         String wallBin = String(walls[x][y], BIN);
    //         while (wallBin.length() < 4) wallBin = "0" + wallBin;
    //         Serial.print(wallBin + " ");
    //     }
    //     Serial.println();
    // }
    
    // Serial.println("\n========================================");
    // Serial.println("DISTANCE ARRAY (by coordinates)");
    // Serial.println("========================================");
    // Serial.print("   ");
    // for (int x = 0; x < MAZE_SIZE; x++) {
    //     Serial.print("  X" + String(x) + " ");
    // }
    // Serial.println();
    
    // for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    //     Serial.print("Y" + String(y) + " ");
    //     for (int x = 0; x < MAZE_SIZE; x++) {
    //         String distStr = (dist[x][y] == 255) ? " -- " : String(dist[x][y]);
    //         if (dist[x][y] < 10 && dist[x][y] != 255) distStr = "  " + distStr;
    //         else if (dist[x][y] < 100 && dist[x][y] != 255) distStr = " " + distStr;
    //         Serial.print(distStr + "  ");
    //     }
    //     Serial.println();
    // }
    
    // Serial.println("\n========================================");
    // Serial.println("       ROBOT POSITION MAP");
    // Serial.println("========================================");
    // Serial.println("LEGEND:");
    // Serial.println("  ^  = Robot facing NORTH (UP)");
    // Serial.println("  >  = Robot facing EAST (RIGHT)");
    // Serial.println("  v  = Robot facing SOUTH (DOWN)");
    // Serial.println("  <  = Robot facing WEST (LEFT)");
    // Serial.println("  T  = TARGET location");
    // Serial.println("  .  = Empty cell");
    // Serial.println("----------------------------------------");
    
    // String dirName;
    // switch(currDir) {
    //     case NORTH: dirName = "NORTH (UP)"; break;
    //     case EAST:  dirName = "EAST (RIGHT)"; break;
    //     case SOUTH: dirName = "SOUTH (DOWN)"; break;
    //     case WEST:  dirName = "WEST (LEFT)"; break;
    // }
    
    // Serial.println("ROBOT at: X=" + String(currX) + ", Y=" + String(currY) + " facing " + dirName);
    // Serial.println("TARGET at: X=" + String(TARGET_X) + ", Y=" + String(TARGET_Y));
    // Serial.println("========================================");
    // Serial.print("   ");
    // for (int x = 0; x < MAZE_SIZE; x++) {
    //     Serial.print(" X" + String(x) + " ");
    // }
    // Serial.println();
    
    // for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    //     Serial.print("Y" + String(y) + " ");
    //     for (int x = 0; x < MAZE_SIZE; x++) {
    //         if (x == currX && y == currY) {
    //             String dirStr;
    //             switch(currDir) {
    //                 case NORTH: dirStr = " ^ "; break;
    //                 case EAST:  dirStr = " > "; break;
    //                 case SOUTH: dirStr = " v "; break;
    //                 case WEST:  dirStr = " < "; break;
    //             }
    //             Serial.print(dirStr);
    //         } else if (x == TARGET_X && y == TARGET_Y) {
    //             Serial.print(" T ");
    //         } else {
    //             Serial.print(" . ");
    //         }
    //     }
    //     Serial.println();
    // }
    // Serial.println("========================================");
    // Serial.println("NOTE: Y increases upward, X increases rightward");
    // Serial.println("========================================\n");
}

void MazeSolver::clearEEPROM()
{
    for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0);
    }
    // Serial.println("EEPROM cleared!");
}