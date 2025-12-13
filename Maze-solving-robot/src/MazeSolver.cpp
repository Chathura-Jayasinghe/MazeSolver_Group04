#include "MazeSolver.h"
#include <EEPROM.h>

// EEPROM Memory Layout Constants
// With 9x9 arrays (81 bytes each):
// Walls:   0-80    (81 bytes)
// Visited: 81-161  (81 bytes)
// Robot position: 162-164 (3 bytes: X, Y, Dir)
// Magic byte: 165
#define EEPROM_START_ADDR 0
#define EEPROM_VISITED_ADDR 81
#define EEPROM_ROBOT_POS_ADDR 162
#define EEPROM_MAGIC_ADDR 165
#define EEPROM_MAGIC_VALUE 0xAA

MazeSolver::MazeSolver(MotorPID &left, MotorPID &right)
    : leftMotor(left), rightMotor(right)
{

    // Initialize Robot at (0,0) facing North
    currX = 0;
    currY = 0;
    currDir = NORTH;

    // Initialize Map: No walls (0), Unknown distance (255)
    for (int x = 0; x <= MAZE_SIZE; x++)
    {
        for (int y = 0; y <= MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            dist[x][y] = 255;
            visited[x][y] = 0;  // Not visited yet
        }
    }
    
    // Mark starting position with its coordinates (0,0 = 00)
    visited[0][0] = (0 * 10) + 0;

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
    updateWalls();
    saveMazeToEEPROM();

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

    // 3. Recalculate Distances (Flood Fill)
    // floodFill();

    // 4. Decide Best Move
    Direction nextDir = getBestDirection();
    Serial.println("Next Direction: " + String(nextDir) + " from Current Direction: " + String(currDir));
    Serial.println("==============================================================\n");

    // 5. Execute Move
    delay(1000);
    turnTo(nextDir);
    moveOneCell();

    // 6. Update Virtual Coordinates
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

// --- Flood Fill Implementation ---
void MazeSolver::floodFill()
{
    // Reset distances
    for (int x = 0; x <= MAZE_SIZE; x++)
    {
        for (int y = 0; y <= MAZE_SIZE; y++)
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
        if (p.y <= MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_NORTH))
        {
            if (dist[p.x][p.y + 1] == 255)
            {
                dist[p.x][p.y + 1] = d + 1;
                queue[tail++] = {p.x, p.y + 1};
            }
        }
        // Check East Neighbor (x+1, y)
        if (p.x <= MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_EAST))
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

Direction MazeSolver::getBestDirectionAt(int x, int y)
{
    int minDist = 255;
    Direction bestDir = NORTH;

    if (y <= MAZE_SIZE - 1 && !(walls[x][y] & WALL_NORTH))
    {
        if (dist[x][y + 1] < minDist)
        {
            minDist = dist[x][y + 1];
            bestDir = NORTH;
        }
    }
    if (x <= MAZE_SIZE - 1 && !(walls[x][y] & WALL_EAST))
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
    for (int steps = 0; steps <= MAZE_SIZE * MAZE_SIZE; steps++)
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
    for (int x = 0; x <= MAZE_SIZE; x++)
    {
        for (int y = 0; y <= MAZE_SIZE; y++)
        {
            walls[x][y] = 0;
            dist[x][y] = 255;
            visited[x][y] = 0;
        }
    }
    visited[0][0] = (0 * 10) + 0;  // Mark starting position
    pathLen = 0;
    pathIndex = 0;
    
    // Clear EEPROM by invalidating magic byte
    EEPROM.write(EEPROM_MAGIC_ADDR, 0x00);
    
    // Optionally clear all EEPROM data
    for (int addr = 0; addr < 132; addr++) {
        EEPROM.write(addr, 0);
    }
    
    Serial.println("EEPROM cleared and system reset!");
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
    while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90)
        ;
    stopMotors();
}

void MazeSolver::turnAround(){
    // turnRight();
    // delay(200);
    // turnRight();
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

// ==================== EEPROM Functions ====================
// EEPROM Memory Layout:
// Address 0-63: walls[8][8] array (64 bytes)
// Address 64-127: visited[8][8] array (64 bytes)
// Address 128: currX (1 byte)
// Address 129: currY (1 byte)
// Address 130: currDir (1 byte)
// Address 131: magic byte (0xAA = valid data saved)

void MazeSolver::saveMazeToEEPROM() {
    Serial.println("\n=== Saving Maze to EEPROM ===");
    
    // Save walls array (64 bytes)
    int addr = EEPROM_START_ADDR;
    for (int x = 0; x <= MAZE_SIZE; x++) {
        for (int y = 0; y <= MAZE_SIZE; y++) {
            EEPROM.write(addr++, walls[x][y]);
        }
    }
    
    // Save visited array (64 bytes)
    addr = EEPROM_VISITED_ADDR;
    for (int x = 0; x <= MAZE_SIZE; x++) {
        for (int y = 0; y <= MAZE_SIZE; y++) {
            EEPROM.write(addr++, visited[x][y]);
        }
    }
    
    // Save robot position and direction
    EEPROM.write(EEPROM_ROBOT_POS_ADDR, currX);
    EEPROM.write(EEPROM_ROBOT_POS_ADDR + 1, currY);
    EEPROM.write(EEPROM_ROBOT_POS_ADDR + 2, currDir);
    
    // Write magic byte to indicate valid data
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    
    Serial.println("Maze saved successfully!");
    Serial.println("Final Position: X=" + String(currX) + " Y=" + String(currY) + " Dir=" + String(currDir));
}

void MazeSolver::loadMazeFromEEPROM() {
    // Check if valid data exists
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
        Serial.println("No saved maze found in EEPROM!");
        return;
    }
    
    Serial.println("\n=== Loading Maze from EEPROM ===");
    
    // Load walls array
    int addr = EEPROM_START_ADDR;
    for (int x = 0; x <= MAZE_SIZE; x++) {
        for (int y = 0; y <= MAZE_SIZE; y++) {
            walls[x][y] = EEPROM.read(addr++);
        }
    }
    
    // Load visited array
    addr = EEPROM_VISITED_ADDR;
    for (int x = 0; x <= MAZE_SIZE; x++) {
        for (int y = 0; y <=    MAZE_SIZE; y++) {
            visited[x][y] = EEPROM.read(addr++);
        }
    }
    
    // Load robot position and direction
    currX = EEPROM.read(EEPROM_ROBOT_POS_ADDR);
    currY = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 1);
    currDir = (Direction)EEPROM.read(EEPROM_ROBOT_POS_ADDR + 2);
    
    Serial.println("Maze loaded successfully!");
    Serial.println("Final Position: X=" + String(currX) + " Y=" + String(currY) + " Dir=" + String(currDir));
}

void MazeSolver::printSavedMaze() {
    // Check if valid data exists
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
        Serial.println("No saved maze found in EEPROM!");
        return;
    }
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║        SAVED MAZE MAP (EEPROM)        ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Print final robot position
    byte savedX = EEPROM.read(EEPROM_ROBOT_POS_ADDR);
    byte savedY = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 1);
    byte savedDir = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 2);
    
    Serial.println("Robot Final Position:");
    Serial.println("  X: " + String(savedX) + " | Y: " + String(savedY));
    String dirName[] = {"NORTH", "EAST", "SOUTH", "WEST"};
    Serial.println("  Direction: " + dirName[savedDir]);
    
    Serial.println("\n────────────────────────────────────────");
    Serial.println("Maze Walls (N=North, E=East, S=South, W=West):\n");
    
    // Print column headers
    Serial.print("    ");
    for (int y = 0; y <= MAZE_SIZE; y++) {
        Serial.print("Y" + String(y) + "   ");
    }
    Serial.println();
    
    // Print maze row by row
    for (int x = 0; x <= MAZE_SIZE; x++) {
        Serial.print("X" + String(x) + " ");
        
        for (int y = 0; y <= MAZE_SIZE; y++) {
            int addr = EEPROM_START_ADDR + (x * (MAZE_SIZE+1)) + y;
            byte wallByte = EEPROM.read(addr);
            
            // Build wall string (show which walls exist)
            String wallStr = "";
            if (wallByte & WALL_NORTH) wallStr += "N";
            if (wallByte & WALL_EAST)  wallStr += "E";
            if (wallByte & WALL_SOUTH) wallStr += "S";
            if (wallByte & WALL_WEST)  wallStr += "W";
            
            if (wallStr == "") wallStr = "----";
            
            // Pad to 5 characters for alignment
            while (wallStr.length() < 4) wallStr += " ";
            Serial.print(wallStr + " ");
        }
        Serial.println();
    }
    
    Serial.println("\n────────────────────────────────────────");
    Serial.println("Binary Wall Data (NESW bits):\n");
    
    // Print binary representation
    Serial.print("    ");
    for (int y = 0; y <=MAZE_SIZE; y++) {
        Serial.print("  Y" + String(y) + "  ");
    }
    Serial.println();
    
    for (int x = 0; x <= MAZE_SIZE; x++) {
        Serial.print("X" + String(x) + " ");
        for (int y = 0; y <= MAZE_SIZE; y++) {
            int addr = EEPROM_START_ADDR + (x * (MAZE_SIZE+1)) + y;
            byte wallByte = EEPROM.read(addr);
            
            // Print as 4-bit binary
            String binary = "";
            for (int bit = 3; bit >= 0; bit--) {
                binary += (wallByte & (1 << bit)) ? "1" : "0";
            }
            Serial.print(binary + "  ");
        }
        Serial.println();
    }
    
    Serial.println("\n────────────────────────────────────────");
    Serial.println("Robot Path (Visited Positions):\n");
    
    // Print visited positions grid
    Serial.print("    ");
    for (int y = 0; y <= MAZE_SIZE; y++) {
        Serial.print("Y" + String(y) + " ");
    }
    Serial.println();
    
    for (int x = 0; x <= MAZE_SIZE; x++) {
        Serial.print("X" + String(x) + "  ");
        for (int y = 0; y <= MAZE_SIZE; y++) {
            int addr = EEPROM_VISITED_ADDR + (x * (MAZE_SIZE+1)) + y;
            byte posValue = EEPROM.read(addr);
            
            if (posValue > 0) {
                // Display the stored position value (e.g., 56 for X=5, Y=6)
                if (posValue < 10) {
                    Serial.print(" " + String(posValue) + " ");
                } else {
                    Serial.print(String(posValue) + " ");
                }
            } else {
                Serial.print(" - ");  // Not visited
            }
        }
        Serial.println();
    }
    
    Serial.println("\nNote: Each cell shows its position (XY format, e.g., 56 = X5,Y6)");
    Serial.println("      '-' means the cell was not visited");
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║            END OF MAZE MAP             ║");
    Serial.println("╚════════════════════════════════════════╝\n");
}