# Multi-Robot Coordination System

A collaborative robotics project implementing autonomous navigation and object transportation using three coordinated robots in the Webots simulator. Each robot is responsible for a specific section of the environment and works together to transport green bottles across the map.

## Project Overview

This system simulates three robots working together to:
1. Detect and collect green bottles
2. Navigate through a sectioned environment
3. Hand off objects between robots
4. Avoid obstacles and walls
5. Return to searching positions

### Robot Roles

- **Robot 1 (Left Section)**: Searches for green bottles and transports them to the handoff point with Robot 2
- **Robot 2 (Middle Section)**: Receives bottles from Robot 1 and hands them off to Robot 3
- **Robot 3 (Right Section)**: Receives bottles from Robot 2 and places them in the final destination area

## Technical Requirements

### Software Requirements
- Webots R2024a or later
- Java Development Kit (JDK) 11 or later
- Webots Java API

### Hardware Used in Simulation
- E-puck robots with the following sensors:
  - Camera (64x64 resolution)
  - Distance sensors (6 sensors)
  - Touch sensors
  - Compass
  - Accelerometer
  - LIDAR (Robot 2 only)
- Gripper attachment for bottle manipulation


## Robot Behaviors

### Robot 1 (ProjectController1.java)
- States:
  - WANDER: Search for green bottles
  - HOME_IN: Approach detected bottle
  - HEAD_TO_GROUND: Lower gripper to grab bottle
  - HEAD_TO_RIGHT: Navigate to handoff point
  - PIVOT: Rotate to transfer position
  - AVOID: Obstacle avoidance
  - PIVOT_TO_POSITION: Align for handoff

### Robot 2 (ProjectController2.java)
- Uses LIDAR for precise positioning
- Maintains position awareness using compass
- Handles bottle transfers between Robot 1 and Robot 3

### Robot 3 (ProjectController3.java)
- Receives bottles from Robot 2
- Navigates to final placement area
- Returns to waiting position after placement

## Key Features

- **Computer Vision**: Uses camera feed to detect green bottles (RGB values)
- **Sensor Fusion**: Combines data from multiple sensors for accurate positioning
- **State Machine**: Each robot implements a state machine for behavior control
- **Obstacle Avoidance**: Uses distance sensors to avoid walls and obstacles
- **Coordinated Handoffs**: Synchronized bottle transfers between robots

## Setup Instructions

1. Install Webots R2024a
2. Clone this repository into your Webots projects directory
3. Open `worlds/robot_world.wbt` in Webots
4. Compile the Java controllers:
   ```bash
   javac -cp "path/to/webots/lib/controller/java/Controller.jar" controllers/*.java
   ```
5. Run the simulation in Webots


## Common Issues and Solutions

1. **Robot Alignment Issues**
   - Ensure compass readings are properly calibrated
   - Check distance sensor thresholds

2. **Missed Bottle Pickups**
   - Adjust RGB detection thresholds
   - Fine-tune gripper positions

3. **Failed Handoffs**
   - Verify robot positioning angles
   - Check timing of gripper operations
