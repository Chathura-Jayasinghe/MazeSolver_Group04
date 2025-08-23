# System Overview – Maze Solving Robot

The Maze Solving Robot consists of three main subsystems: **sensing**, **decision making**, and **actuation**.

## High-Level Flow
1. **Sensors**  
   - IR sensors / ultrasonic sensors to detect walls.
   - Encoders to track movement (optional at this stage).

2. **Decision Making (Algorithm)**  
   - Apply a maze-solving algorithm (e.g., left-hand rule, DFS, BFS).
   - Store and update the map of the maze if needed.

3. **Actuation (Motors)**  
   - Motor drivers control wheels to move the robot.
   - Commands are based on algorithm decisions.

## Simple Block Diagram

```
[Sensors] → [Algorithm / Controller] → [Motors]
```

This simple flow will be refined as we add more detail.