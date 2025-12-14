#include <Arduino.h>
#include <EEPROM.h>

class MotorPID {
private:
    int pin1, pin2;
    int encA, encB;
    bool reversed;
    long encoderCount;
    bool lastA;
    bool direction;
    int speed;

public:
    MotorPID(int p1, int p2, int eA, int eB, bool rev = false) 
        : pin1(p1), pin2(p2), encA(eA), encB(eB), reversed(rev), encoderCount(0), direction(true), speed(0) {}
    
    void begin() {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(encA, INPUT);
        pinMode(encB, INPUT);
        lastA = digitalRead(encA);
        encoderCount = 0;
    }
    
    void setDirection(bool forward) {
        direction = forward;
    }
    
    void setSpeed(int spd) {
        speed = constrain(spd, 0, 255);
        if (speed == 0) {
            digitalWrite(pin1, LOW);
            digitalWrite(pin2, LOW);
        } else {
            bool actualDirection = reversed ? !direction : direction;
            if (actualDirection) {
                analogWrite(pin1, speed);
                digitalWrite(pin2, LOW);
            } else {
                digitalWrite(pin1, LOW);
                analogWrite(pin2, speed);
            }
        }
    }
    
    long getEncoderCount() {
        bool currentA = digitalRead(encA);
        if (currentA != lastA) {
            if (digitalRead(encB) != currentA) {
                encoderCount++;
            } else {
                encoderCount--;
            }
            lastA = currentA;
        }
        return encoderCount;
    }
    
    void resetEncoder() {
        encoderCount = 0;
    }
};

#define MOTOR1_IN1 8
#define MOTOR1_IN2 9
#define ENCODER1_A 3
#define ENCODER1_B 7

#define MOTOR2_IN1 5
#define MOTOR2_IN2 6
#define ENCODER2_A 2 
#define ENCODER2_B 4

#define SWITCH_EXPLORE 31
#define SWITCH_SHORTEST_PATH 33 // Switch 2: ON = Shortest path mode  
#define SWITCH_RESET 35       // Switch 3: ON = Reset memory

enum RobotState {
    IDLE = 0,
    EXPLORE = 1,
    SHORTEST_PATH = 2,
    RESET_MODE = 3
};

enum OperationMode {
    LINE_FOLLOWING = 0,
    WALL_FOLLOWING = 1
};


#define IR1 43
#define IR2 44
#define IR3 45
#define IR4 46
#define IR5 47
#define IR6 48
#define IR7 49
#define IR8 50

#define NUM_SENSORS 8
#define BASE_SPEED_LINE 100
#define TURN_SPEED_LINE 90
#define SEARCH_DURATION_MS 300

enum TurnSearchState_Line {
    NORMAL_FOLLOWING,
    SEARCHING_LEFT,
    SEARCHING_RIGHT,
    TURN_FOUND
};

class LineFollower {
private:
    MotorPID& leftMotor;
    MotorPID& rightMotor;
    
    const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
    const int weights[NUM_SENSORS] = {-3, -2, -1, 0, 0, 1, 2, 3};
    
    const float Kp_line = 30.0;
    const float Kd_line = 0.0;
    float prevError;
    unsigned long prevTime;
    
    TurnSearchState_Line turnState;
    
    void runMotors_Line(int leftPWM, int rightPWM) {
        leftPWM = constrain(leftPWM, 0, 255);
        rightPWM = constrain(rightPWM, 0, 255);

        leftMotor.setDirection(true);
        rightMotor.setDirection(true);
        leftMotor.setSpeed(leftPWM);
        rightMotor.setSpeed(rightPWM);
    }
    
    void turnLeft_line() {
        leftMotor.setDirection(false);
        rightMotor.setDirection(true);
        leftMotor.setSpeed(TURN_SPEED_LINE);
        rightMotor.setSpeed(TURN_SPEED_LINE);
    }
    
    void turnRight_Line() {
        leftMotor.setDirection(true);
        rightMotor.setDirection(false);
        leftMotor.setSpeed(TURN_SPEED_LINE);
        rightMotor.setSpeed(TURN_SPEED_LINE);
    }
    
    void stopMotors() {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }
    
    bool searchForLine(bool searchLeft) {
        unsigned long searchStart = millis();
        
        while (millis() - searchStart < SEARCH_DURATION_MS) {
            if (searchLeft) {
                turnLeft_line();
            } else {
                turnRight_Line();
            }

            delay(10);
            
            if (lineDetected()) {
                return true;
            }
        }
        return false;
    }

public:
    LineFollower(MotorPID& left, MotorPID& right)
        : leftMotor(left), rightMotor(right), 
          prevError(0.0), prevTime(0), turnState(NORMAL_FOLLOWING) {
    }
    
    void begin() {
        for (int i = 0; i < NUM_SENSORS; i++) {
            pinMode(irPins[i], INPUT);
        }
        
        turnState = NORMAL_FOLLOWING;
    }
    
    bool lineDetected() {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (digitalRead(irPins[i]) == HIGH) {
                return true;
            }
        }
        return false;
    }
    
    void update() {
        int sumVal = 0;
        int weightedSum = 0;

        for (int i = 0; i < NUM_SENSORS; i++) {
            int value = digitalRead(irPins[i]);
            if (value == HIGH) {
                weightedSum += weights[i];
                sumVal++;
            }
        }

        switch (turnState) {
            case NORMAL_FOLLOWING: {
                if (sumVal == 0) {
                    turnState = SEARCHING_LEFT;
                    return;
                }

                float error = (float)weightedSum / sumVal;

                unsigned long now = millis();
                float dt = (now - prevTime) / 1000.0;
                float derivative = 0;
                if (dt > 0.0) {
                    derivative = (error - prevError) / dt;
                }

                float correction = Kp_line * error + Kd_line * derivative;

                int leftPWM = BASE_SPEED_LINE - correction;
                int rightPWM = BASE_SPEED_LINE + correction;

                runMotors_Line(leftPWM, rightPWM);

                prevError = error;
                prevTime = now;
                break;
            }

            case SEARCHING_LEFT: {
                if (searchForLine(true)) {
                    turnState = NORMAL_FOLLOWING;
                } else {
                    turnState = SEARCHING_RIGHT;
                    searchForLine(false);
                    delay(100);
                }
                break;
            }

            case SEARCHING_RIGHT: {
                if (searchForLine(false)) {
                    turnState = NORMAL_FOLLOWING;
                } else {
                    // If no line found, continue forward briefly then reset
                    runMotors_Line(BASE_SPEED_LINE, BASE_SPEED_LINE);
                    delay(100);  // Reduced delay
                    turnState = NORMAL_FOLLOWING;
                }
                break;
            }

            default:
                turnState = NORMAL_FOLLOWING;
                break;
        }
        delay(20);
    }
};


#define US_FRONT_TRIG 22
#define US_FRONT_ECHO 24
#define US_LEFT_TRIG 26
#define US_LEFT_ECHO 28
#define US_RIGHT_TRIG 37
#define US_RIGHT_ECHO 36

#define BASE_SPEED      50
#define TURN_SPEED      70
#define WALL_THRESHOLD  15.0f
#define COUNTS_PER_90   250L 
#define COUNTS_PER_180  470L
#define COUNTS_PER_CELL 535L

const float KP = 2.2;
const float KD = 0.0; 
const float DESIRED_WALL_DISTANCE = 7.5;

#define MAZE_SIZE 8
#define TARGET_X  8
#define TARGET_Y  8

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

#define WALL_NORTH (1 << NORTH)
#define WALL_EAST  (1 << EAST)
#define WALL_SOUTH (1 << SOUTH)
#define WALL_WEST  (1 << WEST)

#define EEPROM_START_ADDR 0
#define EEPROM_VISITED_ADDR 81
#define EEPROM_ROBOT_POS_ADDR 162
#define EEPROM_MAGIC_ADDR 165
#define EEPROM_MAGIC_VALUE 0xAA

class MazeSolver {
private:
    MotorPID& leftMotor;
    MotorPID& rightMotor;

    int currX, currY;
    Direction currDir;
    bool CAN_POSSITION_UPDATE;
    
    byte walls[MAZE_SIZE+1][MAZE_SIZE+1];
    int  dist[MAZE_SIZE+1][MAZE_SIZE+1];
    byte visited[MAZE_SIZE+1][MAZE_SIZE+1];

    Direction path[MAZE_SIZE * MAZE_SIZE];
    int pathLen;
    int pathIndex;

public:
    MazeSolver(MotorPID& left, MotorPID& right)
        : leftMotor(left), rightMotor(right) {
        currX = 0;
        currY = 0;
        currDir = NORTH;

        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                walls[x][y] = 0;
                dist[x][y] = 255;
                visited[x][y] = 0;
            }
        }
        
        visited[0][0] = (0 * 10) + 0;
        pathLen = 0;
        pathIndex = 0;
    }
    
    void begin() {
        pinMode(US_FRONT_TRIG, OUTPUT);
        pinMode(US_FRONT_ECHO, INPUT);
        pinMode(US_LEFT_TRIG, OUTPUT);
        pinMode(US_LEFT_ECHO, INPUT);
        pinMode(US_RIGHT_TRIG, OUTPUT);
        pinMode(US_RIGHT_ECHO, INPUT);

        const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
        for (int i = 0; i < NUM_SENSORS; i++) {
            pinMode(irPins[i], INPUT);
        }

        stopMotors();
    }

    float readSensor(int trig, int echo) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        long duration = pulseIn(echo, HIGH, 10000);
        if (duration == 0) return -1;
        return duration * 0.034 / 2;
    }

    void stopMotors() {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }

    void runStep() {
        CAN_POSSITION_UPDATE = true;
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

        if (currX == TARGET_X && currY == TARGET_Y) {
            Serial.println("\n>>> TARGET REACHED! <<<\n");
            
            Serial.println("\n╔════════════════════════════════════════╗");
            Serial.println("║       MAZE EXPLORATION COMPLETE!       ║");
            Serial.println("╚════════════════════════════════════════╝\n");
            
            saveMazeToEEPROM();
            Serial.println(">>> Maze saved to EEPROM!");
            Serial.println(">>> Turn ON Switch 2 for shortest path mode\n");
            
            stopMotors();
            return;
        }

        Direction nextDir = getBestDirectionFromMap();
        Serial.println("Next Direction: " + String(nextDir) + " from Current Direction: " + String(currDir));
        Serial.println("==============================================================\n");

        delay(1000);
        turnTo(nextDir);
        moveOneCell();

        if (CAN_POSSITION_UPDATE) {
            if (currDir == NORTH && currY < MAZE_SIZE)
                currY++;
            else if (currDir == EAST && currX < MAZE_SIZE)
                currX++;
            else if (currDir == SOUTH && currY > 0)
                currY--;
            else if (currDir == WEST && currX > 0)
                currX--;
                
            // Ensure position stays within bounds
            currX = constrain(currX, 0, MAZE_SIZE);
            currY = constrain(currY, 0, MAZE_SIZE);
        }
        
        visited[currX][currY] = (currX * 10) + currY;
        stopMotors();
        delay(1000);
    }

    bool isFinished() {
        if (currX == TARGET_X && currY == TARGET_Y) {
            return true;
        }
        return false;
    }

    bool isTargetDetectedIR() {
        const int irPins[NUM_SENSORS] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (digitalRead(irPins[i]) == HIGH) {
                return true;
            }
        }
        return false;
    }

    void computeShortestPath() {
        floodFill();
        int x = 0, y = 0;
        Direction d;
        pathLen = 0;
        pathIndex = 0;

        if (dist[x][y] == 255) return;

        for (int steps = 0; steps <= MAZE_SIZE * MAZE_SIZE; steps++) {
            if (x == TARGET_X && y == TARGET_Y) break;
            d = getBestDirectionAt(x, y);
            path[pathLen++] = d;
            if (d == NORTH) y++;
            else if (d == EAST) x++;
            else if (d == SOUTH) y--;
            else if (d == WEST) x--;
        }
    }

    void followShortestPathStep() {
        if (pathIndex >= pathLen) {
            stopMotors();
            return;
        }
        Direction nextDir = path[pathIndex];
        turnTo(nextDir);
        moveOneCell();
        if (currDir == NORTH) currY++;
        else if (currDir == EAST) currX++;
        else if (currDir == SOUTH) currY--;
        else if (currDir == WEST) currX--;
        stopMotors();
        pathIndex++;
    }

    void reset() {
        currX = 0;
        currY = 0;
        currDir = NORTH;
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                walls[x][y] = 0;
                dist[x][y] = 255;
                visited[x][y] = 0;
            }
        }
        visited[0][0] = (0 * 10) + 0;
        pathLen = 0;
        pathIndex = 0;
        
        EEPROM.write(EEPROM_MAGIC_ADDR, 0x00);
        for (int addr = 0; addr < 132; addr++) {
            EEPROM.write(addr, 0);
        }
        
        Serial.println("EEPROM cleared and system reset!");
        stopMotors();
    }

    void saveMazeToEEPROM() {
        Serial.println("\n=== Saving Maze to EEPROM ===");
        
        int addr = EEPROM_START_ADDR;
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                EEPROM.write(addr++, walls[x][y]);
            }
        }
        
        addr = EEPROM_VISITED_ADDR;
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                EEPROM.write(addr++, visited[x][y]);
            }
        }
        
        EEPROM.write(EEPROM_ROBOT_POS_ADDR, currX);
        EEPROM.write(EEPROM_ROBOT_POS_ADDR + 1, currY);
        EEPROM.write(EEPROM_ROBOT_POS_ADDR + 2, currDir);
        EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
        
        Serial.println("Maze saved successfully!");
        Serial.println("Final Position: X=" + String(currX) + " Y=" + String(currY) + " Dir=" + String(currDir));
    }

    void loadMazeFromEEPROM() {
        if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
            Serial.println("No saved maze found in EEPROM!");
            return;
        }
        
        Serial.println("\n=== Loading Maze from EEPROM ===");
        
        int addr = EEPROM_START_ADDR;
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                walls[x][y] = EEPROM.read(addr++);
            }
        }
        
        addr = EEPROM_VISITED_ADDR;
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                visited[x][y] = EEPROM.read(addr++);
            }
        }
        
        currX = EEPROM.read(EEPROM_ROBOT_POS_ADDR);
        currY = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 1);
        currDir = (Direction)EEPROM.read(EEPROM_ROBOT_POS_ADDR + 2);
        
        Serial.println("Maze loaded successfully!");
        Serial.println("Final Position: X=" + String(currX) + " Y=" + String(currY) + " Dir=" + String(currDir));
    }

    void printSavedMaze() {
        if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
            Serial.println("No saved maze found in EEPROM!");
            return;
        }
        
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║        SAVED MAZE MAP (EEPROM)        ║");
        Serial.println("╚════════════════════════════════════════╝\n");
        
        byte savedX = EEPROM.read(EEPROM_ROBOT_POS_ADDR);
        byte savedY = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 1);
        byte savedDir = EEPROM.read(EEPROM_ROBOT_POS_ADDR + 2);
        
        Serial.println("Robot Final Position:");
        Serial.println("  X: " + String(savedX) + " | Y: " + String(savedY));
        String dirName[] = {"NORTH", "EAST", "SOUTH", "WEST"};
        Serial.println("  Direction: " + dirName[savedDir]);
        
        Serial.println("\n────────────────────────────────────────");
        Serial.println("Maze Walls (N=North, E=East, S=South, W=West):\n");
        
        Serial.print("    ");
        for (int y = 0; y <= MAZE_SIZE; y++) {
            Serial.print("Y" + String(y) + "   ");
        }
        Serial.println();
        
        for (int x = 0; x <= MAZE_SIZE; x++) {
            Serial.print("X" + String(x) + " ");
            
            for (int y = 0; y <= MAZE_SIZE; y++) {
                int addr = EEPROM_START_ADDR + (x * (MAZE_SIZE+1)) + y;
                byte wallByte = EEPROM.read(addr);
                
                String wallStr = "";
                if (wallByte & WALL_NORTH) wallStr += "N";
                if (wallByte & WALL_EAST)  wallStr += "E";
                if (wallByte & WALL_SOUTH) wallStr += "S";
                if (wallByte & WALL_WEST)  wallStr += "W";
                
                if (wallStr == "") wallStr = "----";
                
                while (wallStr.length() < 4) wallStr += " ";
                Serial.print(wallStr + " ");
            }
            Serial.println();
        }
    }
    
private:
    void floodFill() {
        for (int x = 0; x <= MAZE_SIZE; x++) {
            for (int y = 0; y <= MAZE_SIZE; y++) {
                dist[x][y] = 255;
            }
        }

        struct Point { int x, y; };
        Point queue[MAZE_SIZE * MAZE_SIZE];
        int head = 0, tail = 0;

        dist[TARGET_X][TARGET_Y] = 0;
        queue[tail++] = {TARGET_X, TARGET_Y};

        while (head != tail) {
            Point p = queue[head++];
            int d = dist[p.x][p.y];

            if (p.y <= MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_NORTH)) {
                if (dist[p.x][p.y + 1] == 255) {
                    dist[p.x][p.y + 1] = d + 1;
                    queue[tail++] = {p.x, p.y + 1};
                }
            }
            if (p.x <= MAZE_SIZE - 1 && !(walls[p.x][p.y] & WALL_EAST)) {
                if (dist[p.x + 1][p.y] == 255) {
                    dist[p.x + 1][p.y] = d + 1;
                    queue[tail++] = {p.x + 1, p.y};
                }
            }
            if (p.y > 0 && !(walls[p.x][p.y] & WALL_SOUTH)) {
                if (dist[p.x][p.y - 1] == 255) {
                    dist[p.x][p.y - 1] = d + 1;
                    queue[tail++] = {p.x, p.y - 1};
                }
            }
            if (p.x > 0 && !(walls[p.x][p.y] & WALL_WEST)) {
                if (dist[p.x - 1][p.y] == 255) {
                    dist[p.x - 1][p.y] = d + 1;
                    queue[tail++] = {p.x - 1, p.y};
                }
            }
        }
    }

    void updateWalls() {
        float f = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        float l = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);
        float r = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);

        bool wallFront = (f > 0 && f < 12);
        bool wallLeft = (l > 0 && l < 12);
        bool wallRight = (r > 0 && r < 12);

        if (wallFront) {
            if (currDir == NORTH) walls[currX][currY] |= WALL_NORTH;
            if (currDir == EAST) walls[currX][currY] |= WALL_EAST;
            if (currDir == SOUTH) walls[currX][currY] |= WALL_SOUTH;
            if (currDir == WEST) walls[currX][currY] |= WALL_WEST;
        }
        if (wallRight) {
            if (currDir == NORTH) walls[currX][currY] |= WALL_EAST;
            if (currDir == EAST) walls[currX][currY] |= WALL_SOUTH;
            if (currDir == SOUTH) walls[currX][currY] |= WALL_WEST;
            if (currDir == WEST) walls[currX][currY] |= WALL_NORTH;
        }
        if (wallLeft) {
            if (currDir == NORTH) walls[currX][currY] |= WALL_WEST;
            if (currDir == EAST) walls[currX][currY] |= WALL_NORTH;
            if (currDir == SOUTH) walls[currX][currY] |= WALL_EAST;
            if (currDir == WEST) walls[currX][currY] |= WALL_SOUTH;
        }
    }

    bool hasWallAbsolute(int x, int y, int absDir) {
        if (absDir == NORTH) return walls[x][y] & WALL_NORTH;
        if (absDir == EAST)  return walls[x][y] & WALL_EAST;
        if (absDir == SOUTH) return walls[x][y] & WALL_SOUTH;
        if (absDir == WEST)  return walls[x][y] & WALL_WEST;
        return true;
    }

    Direction getBestDirectionFromMap() {
        Direction bestDir = currDir;
        int frontAbs = currDir;
        int leftAbs  = (currDir + 3) % 4;
        int rightAbs = (currDir + 1) % 4;

        bool wallFront = hasWallAbsolute(currX, currY, frontAbs);
        bool wallLeft  = hasWallAbsolute(currX, currY, leftAbs);
        bool wallRight = hasWallAbsolute(currX, currY, rightAbs);

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

    Direction getBestDirectionAt(int x, int y) {
        int minDist = 255;
        Direction bestDir = NORTH;

        if (y <= MAZE_SIZE - 1 && !(walls[x][y] & WALL_NORTH)) {
            if (dist[x][y + 1] < minDist) {
                minDist = dist[x][y + 1];
                bestDir = NORTH;
            }
        }
        if (x <= MAZE_SIZE - 1 && !(walls[x][y] & WALL_EAST)) {
            if (dist[x + 1][y] < minDist) {
                minDist = dist[x + 1][y];
                bestDir = EAST;
            }
        }
        if (y > 0 && !(walls[x][y] & WALL_SOUTH)) {
            if (dist[x][y - 1] < minDist) {
                minDist = dist[x][y - 1];
                bestDir = SOUTH;
            }
        }
        if (x > 0 && !(walls[x][y] & WALL_WEST)) {
            if (dist[x - 1][y] < minDist) {
                minDist = dist[x - 1][y];
                bestDir = WEST;
            }
        }
        return bestDir;
    }

    void turnTo(Direction targetDir) {
        int diff = (targetDir - currDir);

        if (diff == 0) {
        } else if (diff == 1 || diff == -3) {
            turnRight();
        } else if (diff == -1 || diff == 3) {
            turnLeft();
        } else if (diff == 2 || diff == -2) {
            turnAround();
        }
        currDir = targetDir;
    }

    void moveOneCell() {
        encZero();
        leftMotor.setDirection(true);
        rightMotor.setDirection(true);

        float error = 0.0f;
        float lastError = 0.0f;
        float correction = 0.0f;

        float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
        if (frontDist > 0 && frontDist < 10.0) {
            CAN_POSSITION_UPDATE = false;
            return;
        }

        leftMotor.setSpeed(BASE_SPEED);
        rightMotor.setSpeed(BASE_SPEED);

        while (true) {
            long avg = (labs(encLeft()) + labs(encRight())) / 2;
            if (avg >= COUNTS_PER_CELL) break;

            float rightDist = readSensor(US_RIGHT_TRIG, US_RIGHT_ECHO);
            float leftDist = readSensor(US_LEFT_TRIG, US_LEFT_ECHO);

            bool rightWallExists = (rightDist > 0 && rightDist < WALL_THRESHOLD * 1.2);
            bool leftWallExists = (leftDist > 0 && leftDist < WALL_THRESHOLD * 1.2);

            if (rightWallExists && leftWallExists) {
                error = leftDist - rightDist;
            } else if (rightWallExists) {
                error = DESIRED_WALL_DISTANCE - rightDist;
            } else if (leftWallExists) {
                error = leftDist - DESIRED_WALL_DISTANCE;
            } else {
                error = 0;
            }

            correction = KP * error + KD * (error - lastError);
            lastError = error;
            correction = constrain(correction, -20, 20);
            int leftSpeed = BASE_SPEED - correction;
            int rightSpeed = BASE_SPEED + correction;

            leftSpeed = constrain(leftSpeed, 0, 200);
            rightSpeed = constrain(rightSpeed, 0, 200);

            leftMotor.setSpeed(leftSpeed);
            rightMotor.setSpeed(rightSpeed);

            float frontDist = readSensor(US_FRONT_TRIG, US_FRONT_ECHO);
            if (frontDist > 0 && frontDist < 3.50) {
                break;
            }

            delay(10);
        }
        stopMotors();
    }

    void turnLeft() {
        encZero();
        leftMotor.setDirection(false);
        rightMotor.setDirection(true);
        leftMotor.setSpeed(TURN_SPEED);
        rightMotor.setSpeed(TURN_SPEED);
        while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90);
        stopMotors();
    }

    void turnRight() {
        encZero();
        leftMotor.setDirection(true);
        rightMotor.setDirection(false);
        leftMotor.setSpeed(TURN_SPEED);
        rightMotor.setSpeed(TURN_SPEED);
        while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_90);
        stopMotors();
    }

    void turnAround() {
        encZero();
        leftMotor.setDirection(true);
        rightMotor.setDirection(false);
        leftMotor.setSpeed(TURN_SPEED);
        rightMotor.setSpeed(TURN_SPEED);
        while ((labs(encLeft()) + labs(encRight())) / 2 < COUNTS_PER_180);
        stopMotors();
    }

    long encLeft() { return leftMotor.getEncoderCount(); }
    long encRight() { return rightMotor.getEncoderCount(); }
    void encZero() {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }
};

MotorPID leftMotor(MOTOR1_IN1, MOTOR1_IN2, ENCODER1_A, ENCODER1_B, true);
MotorPID rightMotor(MOTOR2_IN1, MOTOR2_IN2, ENCODER2_A, ENCODER2_B, true);

MazeSolver mazeSolver(leftMotor, rightMotor);
LineFollower lineFollower(leftMotor, rightMotor);

RobotState currentState = IDLE;
RobotState previousState = IDLE;
bool isFinished = true;
bool pathFollowingMode = false;
OperationMode operationMode = LINE_FOLLOWING;

RobotState readSwitchState();
void printCurrentState(RobotState state);
void handleStateChange();
bool checkMazeEntrance();

void setup() {
    Serial.begin(9600);
    
    leftMotor.begin();
    rightMotor.begin();
    mazeSolver.begin();
    lineFollower.begin();

    pinMode(SWITCH_EXPLORE, INPUT_PULLUP);
    pinMode(SWITCH_SHORTEST_PATH, INPUT_PULLUP);
    pinMode(SWITCH_RESET, INPUT_PULLUP);

    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║     FLOOD FILL MAZE SOLVER v3.0       ║");
    Serial.println("║        3-Switch Controlled             ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Switch Controls:");
    Serial.println("  Switch 1 ON  - EXPLORE MAZE MODE");
    Serial.println("  Switch 2 ON  - SHORTEST PATH MODE");
    Serial.println("  Switch 3 ON  - RESET MEMORY");
    Serial.println("  All OFF      - IDLE/STANDBY");
    Serial.println();
    Serial.println("Note: Only turn ON one switch at a time!");
    Serial.println();
    Serial.println("Serial Commands (still available):");
    Serial.println("  Send 'R' - View saved maze from EEPROM");
    Serial.println();
    
    currentState = readSwitchState();
    printCurrentState(currentState);
}

void loop() {
    mazeSolver.runStep();
    // RobotState newState = readSwitchState();
    
    // if (newState != currentState) {
    //     previousState = currentState;
    //     currentState = newState;
    //     handleStateChange();
    // }
    
    // if (Serial.available() > 0) {
    //     char command = Serial.read();
        
    //     if (command == 'R' || command == 'r') {
    //         Serial.println("\n>>> Retrieving saved maze from EEPROM...\n");
    //         mazeSolver.printSavedMaze();
    //         Serial.println(">>> Ready for next command...\n");
    //         return;
    //     }
    // }
    
    // switch (currentState) {
    //     case IDLE:
    //         if (!isFinished) {
    //             isFinished = true;
    //             pathFollowingMode = false;
    //             mazeSolver.stopMotors();
    //         }
    //         break;
            
    //     case EXPLORE:
    //         if (!isFinished) {
    //             if (operationMode == LINE_FOLLOWING) {
    //                 // Follow line until maze entrance detected
    //                 lineFollower.update();
                    
    //                 // Check for maze entrance
    //                 if (checkMazeEntrance()) {
    //                     operationMode = WALL_FOLLOWING;
    //                     Serial.println(">>> Maze entrance detected! Switching to wall following mode");
    //                     mazeSolver.stopMotors();
    //                     delay(500);  // Brief pause for transition
    //                 }
    //             } else {
    //                 // mazeSolver.runStep();
                    
    //                 // if (mazeSolver.isFinished()) {
    //                 //     isFinished = true;
    //                 // }
    //             }
    //         }
    //         break;
            
    //     case SHORTEST_PATH:
    //         if (!isFinished) {
    //             if (operationMode == LINE_FOLLOWING) {
    //                 // Follow line until maze entrance detected
    //                 lineFollower.update();
                    
    //                 // Check for maze entrance
    //                 if (checkMazeEntrance()) {
    //                     operationMode = WALL_FOLLOWING;
    //                     Serial.println(">>> Maze entrance detected! Switching to shortest path mode");
                        
    //                     Serial.println(">>> Computing shortest path...");
    //                     mazeSolver.computeShortestPath();
    //                     pathFollowingMode = true;
    //                     Serial.println(">>> Following shortest path...\n");
    //                     delay(500);  // Brief pause for transition
    //                 }
    //             } else {
    //                 mazeSolver.followShortestPathStep();
                    
    //                 if (mazeSolver.isFinished()) {
    //                     isFinished = true;
    //                     pathFollowingMode = false;
                        
    //                     Serial.println("\n╔════════════════════════════════════════╗");
    //                     Serial.println("║      SHORTEST PATH COMPLETE!           ║");
    //                     Serial.println("╚════════════════════════════════════════╝\n");
    //                     Serial.println(">>> Mission accomplished!\n");
    //                 }
    //             }
    //         }
    //         break;
            
    //     case RESET_MODE:
    //         break;
    // }
}



RobotState readSwitchState() {
    bool switchExplore = !digitalRead(SWITCH_EXPLORE);
    bool switchShortestPath = !digitalRead(SWITCH_SHORTEST_PATH); // Invert because of pullup
    bool switchReset = !digitalRead(SWITCH_RESET);              // Invert because of pullup
    
    if (switchReset) return RESET_MODE;
    if (switchExplore) return EXPLORE;
    // if (switchShortestPath) return SHORTEST_PATH;
    
    return IDLE;
}

void printCurrentState(RobotState state) {
    Serial.print(">>> Current State: ");
    switch (state) {
        case IDLE:
            Serial.println("IDLE/STANDBY");
            Serial.println("    All switches OFF - Waiting...");
            break;
        case EXPLORE:
            Serial.println("EXPLORE MAZE");
            Serial.println("    Switch 1 ON - Starting exploration...");
            break;
        case SHORTEST_PATH:
            Serial.println("SHORTEST PATH");
            Serial.println("    Switch 2 ON - Following shortest path...");
            break;
        case RESET_MODE:
            Serial.println("RESET MEMORY");
            Serial.println("    Switch 3 ON - Clearing memory...");
            break;
    }
    Serial.println();
}

void handleStateChange() {
    printCurrentState(currentState);
    
    switch (currentState) {
        case IDLE:
            if (!isFinished) {
                isFinished = true;
                pathFollowingMode = false;
                mazeSolver.stopMotors();
                Serial.println(">>> Operations stopped - Now in standby");
            }
            break;
            
        case EXPLORE:
            if (isFinished) {
                Serial.println(">>> Starting maze exploration in 2 seconds...");
                Serial.println(">>> Initial mode: LINE FOLLOWING");
                delay(2000);
                isFinished = false;
                pathFollowingMode = false;
                operationMode = LINE_FOLLOWING;  // Always start with line following
                
                mazeSolver.reset();
                Serial.println(">>> Exploration started!\n");
            }
            break;
            
        case SHORTEST_PATH:
            if (isFinished) {
                Serial.println(">>> Loading saved maze from EEPROM...");
                mazeSolver.loadMazeFromEEPROM();
                
                Serial.println(">>> Starting shortest path in 2 seconds...");
                Serial.println(">>> Initial mode: LINE FOLLOWING");
                delay(2000);
                isFinished = false;
                pathFollowingMode = false;
                operationMode = LINE_FOLLOWING;  // Always start with line following
                
                Serial.println(">>> Shortest path mode started!\n");
            }
            break;
            
        case RESET_MODE:
            Serial.println(">>> Clearing EEPROM and resetting memory...");
            mazeSolver.reset();
            isFinished = true;
            pathFollowingMode = false;
            operationMode = LINE_FOLLOWING;
            
            Serial.println(">>> Memory cleared!");
            Serial.println(">>> Turn OFF Switch 3 and turn ON desired mode switch\n");
            delay(1000);
            break;
    }
}

bool checkMazeEntrance() {
    bool allBlack = lineFollower.lineDetected();

    if (!allBlack) {
        return false;
    }

    float leftDistance = mazeSolver.readSensor(26, 28);
    float rightDistance = mazeSolver.readSensor(37, 36);

    bool wallsDetected =
        (leftDistance > 0 && leftDistance <= 15) &&
        (rightDistance > 0 && rightDistance <= 15);

    if (wallsDetected) {
        Serial.print(">>> Maze entrance detected - Left: ");
        Serial.print(leftDistance);
        Serial.print("cm, Right: ");
        Serial.print(rightDistance);
        Serial.println("cm");
        return true;
    }
    return false;
}
