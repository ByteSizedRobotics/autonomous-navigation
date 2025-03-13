# Autonomous Navigation (AutoNav)

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
- Lidar sensor connected via USB

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

## Usage

### Running Obstacle Detection

1. Ensure your Lidar sensor is properly connected

2. Set appropriate permissions for the USB device:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

3. Launch the obstacle detection nodes:
   ```bash
   ros2 launch obstacle_detection obstacle_detector.launch.py use_rviz:=true
   ```
   use_rviz is set to true for development. It is set to false by default.

### NMEA GPS Driver installation

1. Install and set up ROS2-Jazzy
3. Install python3 (If not done already)
4. sudo apt update
5. sudo apt install ros-Jazzy-tf-transformations
6. Move driver into a directory or pull from github
7. Build node: colcon build --packages-select nmea_navsat_driver
8. Make sure GPS unit is on correct port (usually /dev/ttyUSB01)
9. source install/setup.bash
10. ros2 run nmea_navsat_driver nmea_serial_driver

*Note, BN-220 runs at 9600 baud

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
