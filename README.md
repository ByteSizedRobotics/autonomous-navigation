# Autonomous Navigation

![Version](https://img.shields.io/badge/version-0.1-blue)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)

ROS2 autonomous navigation package for GPS waypoint following and obstacle avoidance.

## Nodes

### 1. Simple Waypoint Follower (`simple_waypoint_follower`)
Autonomous GPS waypoint navigation with obstacle avoidance.
- **Subscribes to:**
  - `/fix` (NavSatFix) - GPS position
  - `/imu/data` (Imu) - Heading/orientation
  - `/scan` (LaserScan) - LiDAR obstacle detection
  - `/gps_waypoints` (Float64MultiArray) - Waypoint list [lat1, lon1, lat2, lon2, ...]
- **Publishes to:**
  - `/cmd_vel` (Twist) - Velocity commands
- **Features:**
  - Follows GPS waypoints in sequence
  - Obstacle avoidance using LiDAR
  - Adjusts speed based on heading error
  - Stops at final waypoint

### 2. Simple Autonomous Navigation (`simple_autonomous_navigation`)
Basic autonomous navigation without GPS waypoints.
- **Subscribes to:**
  - `/scan` (LaserScan) - LiDAR obstacle detection
- **Publishes to:**
  - `/cmd_vel` (Twist) - Velocity commands
- **Features:**
  - Moves forward in straight line when path is clear
  - Turns to avoid obstacles
  - No GPS or IMU required

### 3. Rover Serial Bridge (`rover_serial_bridge`)
Converts ROS2 Twist commands to rover motor commands and publishes IMU data.
- **Subscribes to:**
  - `/cmd_vel` (Twist) - Velocity commands from autonomous navigation
  - `/JSON` (String) - Direct motor commands from UI for manual control
- **Publishes to:**
  - `/imu/data` (Imu) - IMU sensor data
  - `/imu/raw` (Float32MultiArray) - Raw IMU values [roll, pitch, yaw, temp, voltage]
- **Features:**
  - Differential drive conversion (Twist → left/right wheel speeds)
  - Serial communication with rover hardware (`{"T": 1, "L": left, "R": right}`)
  - IMU data streaming at 50 Hz
  - Safety watchdog (sends stop commands if no recent messages)

### 4. Rover Command Centre (`rover_command_centre`)
Central hub for managing all rover subsystems.
- **Subscribes to:**
  - `/command` (String) - Commands from external software (LaunchRover, ManualControl, Stop)
  - `/heartbeat` (String) - Keepalive from external software
- **Publishes to:**
  - `/timestamp` (String) - System timestamp when operational
  - `/node_status` (String) - Status of all nodes (JSON)
- **Features:**
  - Launches/stops GPS, LiDAR, cameras, and navigation nodes
  - Monitors node health
  - Emergency stop on heartbeat loss
  - Process management and cleanup

## Launch

### Using Rover Command Centre (Recommended)
The command centre manages all subsystems including GPS and LiDAR:

```bash
# Start the command centre
ros2 run command_centre rover_command_centre

# Send commands via ROS topics or external UI
# - LaunchRover: Starts GPS, LiDAR, cameras, and autonomous navigation
# - ManualControl: Starts sensors for manual control via UI
# - Stop: Stops all nodes safely
```

The command centre automatically launches:
- **GPS**: `ros2 run nmea_navsat_driver nmea_serial_driver`
- **LiDAR**: `ros2 launch obstacle_detection obstacle_detector.launch.py` (includes rplidar)
- **Cameras**: CSI and USB camera nodes
- **Navigation**: `simple_waypoint_follower` or rover control

### Manual Launch (For Testing)

#### Launch Waypoint Follower Navigation
```bash
# Terminal 1: Start GPS
ros2 run nmea_navsat_driver nmea_serial_driver

# Terminal 2: Start LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# Terminal 3: Start navigation stack
ros2 launch autonomous_navigation autonomous_navigation.launch.py
```
This launches:
- `simple_waypoint_follower` - GPS waypoint navigation
- `rover_serial_bridge` - Motor control and IMU

#### Launch Simple Autonomous Navigation (No GPS)
```bash
# Terminal 1: Start LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# Terminal 2: Start navigation
ros2 run autonomous_navigation simple_autonomous_navigation
ros2 run autonomous_navigation rover_serial_bridge
```

### Configuration
Edit parameters in `config/waypoint_follower.yaml`:
```yaml
simple_waypoint_follower:
  ros__parameters:
    max_linear_speed: 0.1      # m/s - Forward speed
    max_angular_speed: 0.2     # rad/s - Turning speed
    waypoint_threshold: 2.0    # meters - Distance to consider waypoint reached
    obstacle_distance: 0.5     # meters - Stop distance from obstacles
    obstacle_angle: 60.0       # degrees - Front detection sector width

rover_serial_bridge:
  ros__parameters:
    port: '/dev/ttyAMA0'       # Serial port (GPIO pins)
    # port: '/dev/rover_serial' # Or USB port
```

## Sending Waypoints

Waypoints are sent to `/gps_waypoints` as a flat array of alternating latitude/longitude pairs:

```python
from std_msgs.msg import Float64MultiArray

msg = Float64MultiArray()
msg.data = [lat1, lon1, lat2, lon2, lat3, lon3, ...]  # Alternating lat/lon pairs
publisher.publish(msg)
```

Example via command line:
```bash
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray \
  "{data: [45.3158, -75.8378, 45.3159, -75.8379, 45.3160, -75.8380]}"
```

## Manual Control via UI

The rover can be manually controlled through the UI by publishing JSON commands to `/JSON`:
```json
{"T": 1, "L": 50, "R": 50}  // Move forward
{"T": 1, "L": -50, "R": -50}  // Move backward
{"T": 1, "L": 50, "R": -50}  // Turn right
{"T": 1, "L": 0, "R": 0}  // Stop
```

## Important Notes

⚠️ **GPS and LiDAR**: When using the Rover Command Centre, GPS and LiDAR nodes are launched automatically. Manual launch is only needed for testing individual components.

⚠️ **Serial Port Permissions**: Ensure proper permissions for serial devices:
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

⚠️ **Speed Configuration**: Adjust speed multipliers in `rover_serial_bridge.py` if the rover moves too fast/slow:
- Line ~108: `int(left * 1)` - Change multiplier (currently 1)
- Speed ranges depend on your rover's motor controller

## Dependencies
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `pyserial`
- `numpy`
- `tf_transformations`
- `psutil` (for command centre)

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

