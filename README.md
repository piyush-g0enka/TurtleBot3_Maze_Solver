# Turtlebot3 Maze Solver

## Overview

This project is a simulation of a Turtlebot3 robot navigating through a maze environment using ROS2 (Robot Operating System 2). The robot is equipped with an RGB camera for Aruco marker detection and a logical camera for identifying parts in the environment.

![demo](https://github.com/piyush-g0enka/TurtleBot3_Maze_Solver/blob/main/waffle_demo.gif)
## Getting Started

To start the simulation, use the following command:

```bash
ros2 launch turtlebot3_gazebo maze.launch.py
```

This will initialize the Gazebo environment with the Turtlebot3, RGB camera, and logical camera.

## Node(s) Description

### MoveRobot Node

This node is responsible for moving the robot in a straight line and stopping it when it is at a distance ≤ 0.4 m from an Aruco marker. It utilizes the `cmd_vel` topic for robot movement and subscribes to the `odom` topic to get the robot's pose.

### ArucoMarkerHandler Node

This node handles the detection of Aruco markers and performs the following tasks:

1. Transforms the detected Aruco marker into the `odom` frame.
2. Computes the distance between the robot and the marker.
3. Checks the parameter associated with the detected marker's ID for further instructions.

### LogicalCameraHandler Node

This node subscribes to the `mage/advanced_logical_camera/image` topic and detects blue and green batteries. It prints the pose of detected batteries in the `odom` frame, ignoring repeated detections.

## Instructions

1. Start the simulation using the provided launch command.
2. Run the MoveRobot node to navigate towards Aruco markers.
3. ArucoMarkerHandler node checks marker parameters and performs appropriate actions (rotate 90 degrees, etc.).
4. Repeat steps 2-3 until a marker with the parameter value "end" is detected.
5. Once the "end" marker is detected, LogicalCameraHandler node prints the pose of detected batteries in the terminal.

Note: Ensure that you have the required ROS2 packages installed and sourced before running the nodes.

## Dependencies

- ROS2 (Galactic)
- Turtlebot3 Packages
- OpenCV-contrib
- https://github.com/piyush-g0enka/enpm809Y_fall2023/tree/rwa3

## Authors

1. Piyush Goenka
2. Hoang Pham
