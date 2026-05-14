# Autonomous Lane Detection and Following Robot - Unity 3D

## Overview

This project is a Unity 3D simulation of an autonomous mobile robot designed to detect and follow lane markings in a virtual environment. The system uses raycast-based virtual sensors, PID-style steering control, pure-pursuit path tracking, curve prediction, speed control and obstacle avoidance.

The project was developed as my final-year BEng Robotics and Artificial Intelligence project at the University of Hertfordshire.

## Key Features

- Unity 3D physics-based mobile robot simulation
- Raycast-based lane detection using virtual reflective sensors
- Separate left and right lane detection using Unity physics layers
- Median filtering for more stable lane-edge estimation
- PID-style steering controller with anti-windup protection
- Heading correction and pure-pursuit feedforward tracking
- Curve prediction and automatic speed control
- Obstacle detection using sphere casts
- Finite-state obstacle avoidance system
- Telemetry logging for performance evaluation
- Testing across straight, S-curve and obstacle tracks

## Technologies Used

- Unity 3D
- C#
- Unity Rigidbody physics
- Raycasts and sphere casts
- MATLAB for results analysis
- CSV telemetry logging

## How It Works

The robot uses downward-facing raycasts to simulate reflective line sensors. These sensors detect the left and right lane markings and estimate the robot's lateral position relative to the lane centre.

The steering controller combines several components:

- Proportional correction for current lane error
- Integral correction with anti-windup protection
- Derivative damping to reduce oscillation
- Heading correction using a lookahead point
- Pure-pursuit feedforward for smoother curve tracking

For obstacle avoidance, the robot uses sphere casts to detect objects ahead and follows a finite-state sequence:

1. Detect obstacle
2. Dodge around the obstacle
3. Pass the obstacle
4. Merge back into the lane

## Test Environments

The robot was tested on three different track layouts:

1. Straight track
2. S-curve track
3. Obstacle avoidance track

## Results

The final controller achieved stable lane-following performance across all three test environments.

| Track | Mean Absolute Error |
|---|---|
| Straight track | 0.08% |
| S-curve track | 3.6% |
| Obstacle track | 5.0% |

The robot successfully followed lanes, handled curved paths and completed an obstacle avoidance manoeuvre while maintaining stable control.

## Project Structure

```text
Assets/
├── Scripts/
│   ├── RobotController.cs
│   ├── PerformanceLogger.cs
│   ├── CameraShift.cs
│   ├── WheelMovement.cs
│   └── SimpleObjectMovement.cs
│
├── Scenes/
├── Materials/
├── Prefabs/
└── Screenshots/

Packages/
ProjectSettings/
README.md
