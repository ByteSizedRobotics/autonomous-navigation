# Autonomous Navigation (AutoNav)

![Version](https://img.shields.io/badge/version-0.1-blue)
![ROS](https://img.shields.io/badge/ROS-compatible-green)

A comprehensive solution for autonomous navigation using ROS, featuring obstacle detection, path planning, and more.

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
- ROS1 Noetic
- Ubuntu 20.04 (recommended)
- Lidar sensor connected via USB

### Setup Instructions

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/rover_ws/src
   git clone https://github.com/ByteSizedRobotics/autonomous-navigation.git
   ```

2. Navigate to your workspace directory:
   ```bash
   cd ~/rover_ws
   ```

3. Build the packages:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running Obstacle Detection

1. Ensure your Lidar sensor is properly connected

2. Set appropriate permissions for the USB device:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

3. Launch the obstacle detection nodes:
   ```bash
   ros2 launch obstacle_detection obstacle_detector.launch.py
   ```

### Expected Output

When running successfully, you should see output similar to:

![Obstacle Detection Output](https://github.com/user-attachments/assets/87bf6039-ba21-4f54-9947-c46dac4db427)

## Configuration

The detection radius and other parameters can be configured in the `config/obstacle_detector_params.yaml` file.

## Project Roadmap

- [x] V0.1: Basic obstacle detection
- [ ] V0.2: Improved obstacle detection
- [ ] V0.3: Path planning
- [ ] V1.0: Full autonomous navigation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
