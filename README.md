# Double Pendulum Robot Arm Controller

<img src="resources/screenshot2.png" width="600">

<!-- ![Image](resources/demo.png) -->

## Overview
A two-linked robot arm simulated as a double pendulum with Langrangian equations of motion. Through inverse kinematics the joint positions to reach the mouse position on the screen are calculated. A non-linear controller defines the dynamics with which the endeffector approached the desired position. The controller can be turned off, to let the robot act as a double pendulum.

## Installation
Dependencies:
```
sudo apt install libsdl2-dev libsdl2-ttf-dev
```
Build:
```
git clone https://github.com/LennardMarx/RobotArm.git
cd RobotArm
chmod +x build.sh
./build.sh
```
Run with:
```
./bin/robotarm
```
Install globally with:
```
sudo cp bin/robotarm /usr/local/bin/robotarm
```

## Controls
Mouse - define the desired endeffector position
C - toggle controller
T - toggle the trail drawn by the endeffector

## Details

The Robot is a double pendulum equipped with two motors, one at the base and one at the joint of the links. When the controller is deactivated, it will swing freely under the physics applied with regard to the weight and size defined. The joints are 50 cm long and weight 1 kg each. The joints are damped to simulate a resistance from the motors and gears.

The endeffector follows the mouse pointer under a defined control law.

Two keys can be pressed to influence the behavior of the program:

- T: Activate/deactivate the trace the endeffector draws on the screen.
- C: Activate/deactivat the controller.

## Equations of motion

- will follow
- test

## Control law (non-linear control)

- will follow
