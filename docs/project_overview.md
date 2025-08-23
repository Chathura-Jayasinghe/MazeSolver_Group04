# Project Overview – Maze Solving Robot

This project aims to design and build a robot capable of autonomously solving a maze.  
The robot will use sensors to detect walls, apply a maze-solving algorithm, and navigate towards the exit.

## Possible Maze-Solving Algorithms

- **Left-hand rule**: Keep the left hand on the wall until the exit is found. Simple but not always optimal.
- **Right-hand rule**: Same as above, but with the right hand.
- **Depth-First Search (DFS)**: Explore as far as possible along one path before backtracking.
- **Breadth-First Search (BFS)**: Explore level by level, ensuring the shortest path is found.
- **Flood Fill**: Commonly used in micromouse competitions for efficient pathfinding.

## Goals for Initial Stage
- Set up basic hardware and PlatformIO environment.
- Represent maze structure in software.
- Test simple algorithms in simulation.
