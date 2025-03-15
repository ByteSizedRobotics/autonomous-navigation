# Autonomous Navigation (AutoNav)
![Version](https://img.shields.io/badge/version-0.1-blue)
![ROS2](https://img.shields.io/badge/ROS2-compatible-green)

A comprehensive solution for autonomous navigation using ROS2, featuring obstacle detection, path planning, and more.

## Overview
This repository contains all code for the Autonomous Navigation (AutoNav) project. The system uses various sensors and algorithms to enable autonomous movement while avoiding obstacles.

## Features
### V0.1 - Obstacle Detection
The initial version implements intelligent obstacle detection using a Lidar sensor.

**Approach:**
- Scans the environment using Lidar sensor data
- Detects obstacles within a configurable forward corridor
- Uses coordinate transformation to focus detection in the rover's direction of travel
- Provides distance measurements to the closest detected obstacle
- Configurable corridor width and length to match rover's path requirements
- Adjustable forward direction to adapt to different mounting orientations

## Topics
The obstacle detector publishes to the following topics:
- `/obstacle_detected` - Boolean indicating if an obstacle is detected in the forward corridor
- `/obstacle_distance` - Float32 with the distance to the closest obstacle (only published when an obstacle is detected)

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

### Configuration
The detector's parameters can be configured in the `config/obstacle_detector_params.yaml` file:
- `distance_threshold`: Detection threshold distance in meters
- `corridor_width`: Width of detection corridor in meters
- `corridor_length`: Length of detection corridor in meters
- `forward_direction`: Forward direction in radians (0=+x axis, π=backward, π/2=left, -π/2=right)
- `enable_debug_output`: Enable/disable detailed debug logging

### Expected Output
When running successfully, you should see output similar to:
<img width="730" alt="Screenshot 2025-03-15 at 6 04 43 PM" src="https://github.com/user-attachments/assets/d4f53a37-00c1-4f95-b2c6-8332267b2764" />

## Project Roadmap
- [x] V0.1: Basic obstacle detection
- [ ] V0.2: Improved obstacle detection
- [ ] V0.3: Path planning
- [ ] V1.0: Full autonomous navigation

## License
This project is licensed under the MIT License - see the LICENSE file for details.
