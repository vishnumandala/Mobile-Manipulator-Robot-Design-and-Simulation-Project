# Mobile Manipulator Robot Design and-Simulation-Project
## ENPM662 - Introduction to Robot Modeling

## Dependencies
- python 3.11 (any version above 3 should work)
- Python running IDE (I used VS Code)
- ROS 2 (Galactic)
- Gazebo 11.14

## Libraries
- Math
- Numpy
- Matplotlib
- Time
- Sympy

## Installation Instructions
1. Download and place the packages in your ROS2 workspace, build and source it. Navigate to the package directory before running the commands.
2. Download the assembly.zip from https://umd0-my.sharepoint.com/:u:/g/personal/vishnum_umd_edu/EZMYW7EYO9lAuvrxkKbrZyYBsPOWxWVwA3TLGkpkDty8Nw?e=f6Znmb
   
## Problem
The primary aim of this project is to create an autonomous robotic system capable of picking and handling tools from a conveyor belt. The system consists of a UR10 manipulator mounted on a mobile chassis with four wheels. The integration of these components was intended to facilitate the autonomous identification and manipulation of objects. However, due to challenges in integrating a vision system, the robot currently operates based on direct commands rather than autonomous decision-making.

## Features
- Teleoperation for navigation
- Forward and Inverse Kinematics Validation
- Picks up a box from the conveyor

## Usage
- Run ros2 launch bot debug.launch.py in one terminal.
- In another terminal, run ros2 run bot bot_controller.py.
- Choose to teleop (t), execute the control loop (c), perform inverse (i) or forward kinematics validation (f).
- Teleop: Use WASD keys for movement. The robot will move in the direction of the key pressed.
- Forward Kinematics Validation: Enter 6 joint angles to view the robot's orientation for those angles.
- Inverse Kinematics Validation: Robot draws an arc of radius 0.5m and executes it for 3.3s. A 3D plot of the trajectory is displayed.
- Control: Robot manipulator follows trajectory to bend to the table, activates gripper and link attacher to grab the box and returns to its home position.

## Results
The project successfully demonstrated a mobile robot's ability to navigate and interact with objects by picking them up.

## Model
![](https://github.com/vishnumandala/Mobile-Manipulator-Robot-Design-and-Simulation-Project/blob/main/model.jpg)

## Workspace
![](https://github.com/vishnumandala/Mobile-Manipulator-Robot-Design-and-Simulation-Project/blob/main/Workspace.jpg)

## Inverse Kinematics Validation
![](https://github.com/vishnumandala/Mobile-Manipulator-Robot-Design-and-Simulation-Project/blob/main/inverse%20validation.gif)

## Simulation
![](https://github.com/vishnumandala/Mobile-Manipulator-Robot-Design-and-Simulation-Project/blob/main/simulation.gif)

## Resources Used
1. IFRA Link Attacher: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher
2. IFRA Conveyor Belt: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt




