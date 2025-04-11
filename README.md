# Autonomous Navigation (Rover)

![Version](https://img.shields.io/badge/version-0.1-blue)
![ROS2](https://img.shields.io/badge/ROS2-compatible-green)

A comprehensive solution for autonomous navigation using ROS2, featuring obstacle detection, path planning, and more.

## Overview

This repository contains all code for the Autonomous Navigation (AutoNav) project. The system uses various sensors and algorithms to enable autonomous movement while avoiding obstacles.

## Features

### V0.1 - Obstacle Detection

The initial version implements basic obstacle detection using a Lidar sensor.

**Approach:**
- Scans the environment using Lidar sensor data
- Detects obstacles by identifying points within a predefined radius
- Simple yet effective algorithm for basic obstacle avoidance

## Installation

### Prerequisites
- ROS2 Jazzy
- Ubuntu 24.04 (recommended)

### Setup Instructions

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ByteSizedRobotics/autonomous-navigation.git
   ```

2. Navigate to your workspace directory:
   ```bash
   cd ~/ros2_ws
   ```

3. Build the packages:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Run the MAIN launch file
```ros2 launch obstacle_detection main.launch.py```

This should launch the current implemented modules (GPS, Manual Control with Front-end, LiDAR, Live-CSI Camera Stream)

