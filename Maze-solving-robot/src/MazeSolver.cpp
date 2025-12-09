#include "MazeSolver.h"



MazeSolver::MazeSolver(MotorPID& left, MotorPID& right)
    : leftMotor(left), rightMotor(right) {
    
    // Initialize Robot at (0,0) facing North
    currX = 0;
    currY = 0;
    currDir = NORTH;

    // Initialize Map: No walls (0), Unknown distance (255)
    for(int x=0; x<MAZE_SIZE; x++) {
        for(int y=0; y<MAZE_SIZE; y++) {
            walls[x][y] = 0;
            dist[x][y]  = 255; 
        }
    }
}

void MazeSolver::begin() {
    pinMode(US_FRONT_TRIG, OUTPUT); 
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);  
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT); 
    pinMode(US_RIGHT_ECHO, INPUT);
    
    stopMotors();
}

// --- Main Algorithm Loop ---
void MazeSolver::runStep() {
    // 1. Read Sensors & Update Walls
    updateWalls();

    // 2. Check if we reached target
    if(currX == TARGET_X && currY == TARGET_Y) {
        stopMotors();
        return; // DONE!
    }

    // 3. Recalculate Distances (Flood Fill)
    floodFill();

    // 4. Decide Best Move
    Direction nextDir = getBestDirection();

    // 5. Execute Move
    turnTo(nextDir);
    moveOneCell();
    
    // 6. Update Virtual Coordinates
    if(currDir == NORTH) currY++;
    else if(currDir == EAST) currX++;
    else if(currDir == SOUTH) currY--;
    else if(currDir == WEST) currX--;
    
    // Small stop to stabilize before next reading
    stopMotors();
    
    delay(200);
}

bool MazeSolver::isFinished() {
    return (currX == TARGET_X && currY == TARGET_Y);
}

// --- Flood Fill Implementation ---
void MazeSolver::floodFill() {
    // Reset distances
    for(int x=0; x<MAZE_SIZE; x++) {
        for(int y=0; y<MAZE_SIZE; y++) {
            dist[x][y] = 255;
        }
    }
    
    // Queue for BFS (Circular buffer)
    struct Point { int x, y; };
    Point queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    // Start at Target
    dist[TARGET_X][TARGET_Y] = 0;
    queue[tail++] = {TARGET_X, TARGET_Y};

    while(head != tail) {
        Point p = queue[head++];
        int d = dist[p.x][p.y];

        // Check North Neighbor (x, y+1)
        if(p.y < MAZE_SIZE-1 && !(walls[p.x][p.y] & WALL_NORTH)) {
            if(dist[p.x][p.y+1] == 255) {
                dist[p.x][p.y+1] = d + 1;
                queue[tail++] = {p.x, p.y + 1};
            }
        }
        // Check East Neighbor (x+1, y)
        if(p.x < MAZE_SIZE-1 && !(walls[p.x][p.y] & WALL_EAST)) {
            if(dist[p.x+1][p.y] == 255) {
                dist[p.x+1][p.y] = d + 1;
                queue[tail++] = {p.x + 1, p.y};
            }
        }
        // Check South Neighbor (x, y-1)
        if(p.y > 0 && !(walls[p.x][p.y] & WALL_SOUTH)) {
            if(dist[p.x][p.y-1] == 255) {
                dist[p.x][p.y-1] = d + 1;
                queue[tail++] = {p.x, p.y - 1};
            }
        }
        // Check West Neighbor (x-1, y)
        if(p.x > 0 && !(walls[p.x][p.y] & WALL_WEST)) {
            if(dist[p.x-1][p.y] == 255) {
                dist[p.x-1][p.y] = d + 1;
                queue[tail++] = {p.x - 1, p.y};
            }
        }
    }
}

// --- Mapping & Sensors ---
void MazeSolver::updateWalls() {
    float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
    float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
    float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

    bool wallFront = (f > 0 && f < DESIRED_WALL_DISTANCE);
    bool wallLeft  = (l > 0 && l < WALL_THRESHOLD);
    bool wallRight = (r > 0 && r < WALL_THRESHOLD);

    // Map Relative (Front/Left/Right) to Absolute (N/E/S/W)
    if(wallFront) {
        if(currDir == NORTH) walls[currX][currY] |= WALL_NORTH;
        if(currDir == EAST)  walls[currX][currY] |= WALL_EAST;
        if(currDir == SOUTH) walls[currX][currY] |= WALL_SOUTH;
        if(currDir == WEST)  walls[currX][currY] |= WALL_WEST;
    }
    if(wallRight) {
        if(currDir == NORTH) walls[currX][currY] |= WALL_EAST;
        if(currDir == EAST)  walls[currX][currY] |= WALL_SOUTH;
        if(currDir == SOUTH) walls[currX][currY] |= WALL_WEST;
        if(currDir == WEST)  walls[currX][currY] |= WALL_NORTH;
    }
    if(wallLeft) {
        if(currDir == NORTH) walls[currX][currY] |= WALL_WEST;
        if(currDir == EAST)  walls[currX][currY] |= WALL_NORTH;
        if(currDir == SOUTH) walls[currX][currY] |= WALL_EAST;
        if(currDir == WEST)  walls[currX][currY] |= WALL_SOUTH;
    }

    // Sync neighbors (If I have North wall, neighbor above has South wall)
    if((walls[currX][currY] & WALL_NORTH) && currY < MAZE_SIZE-1) walls[currX][currY+1] |= WALL_SOUTH;
    if((walls[currX][currY] & WALL_EAST)  && currX < MAZE_SIZE-1) walls[currX+1][currY] |= WALL_WEST;
    if((walls[currX][currY] & WALL_SOUTH) && currY > 0)           walls[currX][currY-1] |= WALL_NORTH;
    if((walls[currX][currY] & WALL_WEST)  && currX > 0)           walls[currX-1][currY] |= WALL_EAST;
}

Direction MazeSolver::getBestDirection() {
    int minDist = 255;
    Direction bestDir = currDir; // Default to straight if equal

    // Check all 4 neighbors. If accessible, check distance.
    
    // NORTH
    if(currY < MAZE_SIZE-1 && !(walls[currX][currY] & WALL_NORTH)) {
        if(dist[currX][currY+1] < minDist) { minDist = dist[currX][currY+1]; bestDir = NORTH; }
    }
    // EAST
    if(currX < MAZE_SIZE-1 && !(walls[currX][currY] & WALL_EAST)) {
        if(dist[currX+1][currY] < minDist) { minDist = dist[currX+1][currY]; bestDir = EAST; }
    }
    // SOUTH
    if(currY > 0 && !(walls[currX][currY] & WALL_SOUTH)) {
        if(dist[currX][currY-1] < minDist) { minDist = dist[currX][currY-1]; bestDir = SOUTH; }
    }
    // WEST
    if(currX > 0 && !(walls[currX][currY] & WALL_WEST)) {
        if(dist[currX-1][currY] < minDist) { minDist = dist[currX-1][currY]; bestDir = WEST; }
    }

    return bestDir;
}

// --- Movement ---
void MazeSolver::turnTo(Direction targetDir) {
    int diff = (targetDir - currDir);
        
    if (diff == 0) {
        // Straight - Do nothing
        Serial.println("straight");
    } else if (diff == 1 || diff == -3) {
        turnRight();
        Serial.println("right");
        // while (true){};
    } else if (diff == -1 || diff == 3) {
        turnLeft();
        Serial.println("left");
        // while (true){};
    } else {
        turnAround();
        Serial.println("u turn");
        // while (true){};
    }
    
    currDir = targetDir;
}

void MazeSolver::moveOneCell() {
    // Move forward one cell (18cm) while following walls using PD control
    // Using ultrasonic sensors on left and right to maintain distance
    
    encZero();
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    
    float error = 0;
    float lastError = 0;
    float correction = 0;

    leftMotor.setSpeed(BASE_SPEED);
    rightMotor.setSpeed(BASE_SPEED);

    while(true) {
        long avg = (labs(encLeft()) + labs(encRight())) / 2;
        if(avg >= COUNTS_PER_CELL) break;
        
        // --- Simple PD Wall Following ---
        float rightDist = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);
        float leftDist = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);

        bool rightWallExists = (rightDist > 0 && rightDist < WALL_THRESHOLD * 1.2);
        bool leftWallExists = (leftDist > 0 && leftDist < WALL_THRESHOLD * 1.2);

        Serial.println("Left wall: " + String(leftDist) + " Right wall: " + String(rightDist) + "front dist: " + String(readSensor(US_FRONT_TRIG, US_FRONT_ECHO)));

        if(rightWallExists && leftWallExists) {
            // Both walls present, try to center
            error = leftDist - rightDist+1;
        } else if (rightWallExists) {
            // Follow right wall
            error = DESIRED_WALL_DISTANCE - rightDist;
        } else if (leftWallExists) {
            // Follow left wall
            error = leftDist - DESIRED_WALL_DISTANCE+1;
        } else {
            // No side walls, go straight
            error = 0;
        }

        correction = KP * error + KD * (error - lastError);
        lastError = error;

        int leftSpeed = BASE_SPEED - correction;
        int rightSpeed = BASE_SPEED + correction;

        // Clamp speeds to valid range
        leftSpeed = constrain(leftSpeed, 0, 200);
        rightSpeed = constrain(rightSpeed, 0, 200);

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
        // --- End PD Wall Following ---
        
        // Safety: If too close to front wall, stop early
        float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if(frontDist > 0 && frontDist < 3.0) break;

        delay(10); // Loop delay for stability
    }
    stopMotors();
   
}

void MazeSolver::turnLeft() {
    encZero();
    leftMotor.setDirection(false); 
    rightMotor.setDirection(true);
    leftMotor.setSpeed(TURN_SPEED); 
    rightMotor.setSpeed(TURN_SPEED);
    while((labs(encLeft()) + labs(encRight()))/2 < COUNTS_PER_90);
    stopMotors();
}

void MazeSolver::turnRight() {
    encZero();
    leftMotor.setDirection(true); 
    rightMotor.setDirection(false);
    leftMotor.setSpeed(TURN_SPEED); 
    rightMotor.setSpeed(TURN_SPEED);
    while((labs(encLeft()) + labs(encRight()))/2 < COUNTS_PER_90);
    stopMotors();
}

void MazeSolver::turnAround() {
    turnRight();
    delay(200);
    turnRight();
}

void MazeSolver::stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

// --- Helpers ---
float MazeSolver::readSensor(int trig, int echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, 10000); 
    if(duration == 0) return -1;
    return duration * 0.034 / 2;
}

long MazeSolver::encLeft() { return leftMotor.getEncoderCount(); }
long MazeSolver::encRight() { return rightMotor.getEncoderCount(); }
void MazeSolver::encZero() { leftMotor.resetEncoder(); rightMotor.resetEncoder(); }