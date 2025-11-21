# Autonomous Navigation Package

A simple ROS2 package for autonomous GPS waypoint following with basic LiDAR obstacle avoidance. Includes integrated rover serial bridge for IMU data and motor control.

## Features

- **GPS Waypoint Following**: Navigates to a series of GPS waypoints sequentially
- **IMU-based Heading**: Uses IMU data for accurate heading information
- **LiDAR Obstacle Avoidance**: Basic obstacle avoidance using LiDAR scan data
- **Continuous Operation**: Does not stop until the final waypoint is reached
- **Rover Serial Bridge**: Handles serial communication with the rover for IMU input and motor control

## Nodes

### 1. simple_waypoint_follower

Main autonomous navigation logic node.

### Subscribed Topics

- `/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)) - GPS position data
- `/imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) - IMU orientation data
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)) - LiDAR scan data
- `/gps_waypoints` ([std_msgs/Float64MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html)) - GPS waypoints as [lat1, lon1, lat2, lon2, ...]

### Published Topics

- `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) - Velocity commands for the robot

### Parameters

- `waypoint_threshold` (double, default: 2.0) - Distance in meters to consider a waypoint reached
- `max_linear_speed` (double, default: 0.5) - Maximum forward speed in m/s
- `max_angular_speed` (double, default: 0.8) - Maximum turning speed in rad/s
- `obstacle_distance` (double, default: 1.5) - Distance threshold for obstacle detection in meters
- `obstacle_angle` (double, default: 60.0) - Front sector angle for obstacle detection in degrees

### 2. rover_serial_bridge

Handles serial communication with the Wave Rover.

#### Subscribed Topics

- `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) - Velocity commands to send to rover
- `/JSON` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)) - Raw JSON commands for rover

#### Published Topics

- `/imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) - IMU orientation data
- `/imu/raw` ([std_msgs/Float32MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html)) - Raw IMU data [roll, pitch, yaw, temp, voltage]

#### Parameters

- `port` (string, default: '/dev/ttyAMA0') - Serial port for rover communication

## System Architecture

The package consists of two nodes that work together:

1. **rover_serial_bridge**: Communicates with the physical rover hardware
   - Reads IMU data from rover and publishes to `/imu/data`
   - Subscribes to `/cmd_vel` and sends motor commands to rover via serial
   - Maintains safety by sending stop commands if no recent velocity commands

2. **simple_waypoint_follower**: Implements autonomous navigation logic
   - Subscribes to GPS (`/fix`), IMU (`/imu/data`), and LiDAR (`/scan`)
   - Computes navigation commands and publishes to `/cmd_vel`
   - Handles waypoint following and obstacle avoidance

```
GPS (/fix) ──────────┐
                      │
LiDAR (/scan) ───────┤
                      ├──> simple_waypoint_follower ──> /cmd_vel ──> rover_serial_bridge ──> Rover Hardware
IMU (/imu/data) ─────┤                                                      ↑
                      │                                                      │
Waypoints ───────────┘                                          Serial Communication
(/gps_waypoints)
```

## Usage

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_navigation
source install/setup.bash
```

### Running the Complete System

Launch both nodes together (recommended):

```bash
ros2 launch autonomous_navigation autonomous_navigation.launch.py
```

With custom serial port:

```bash
ros2 launch autonomous_navigation autonomous_navigation.launch.py serial_port:=/dev/ttyUSB0
```

### Running Individual Nodes

Launch only the waypoint follower:

```bash
ros2 launch autonomous_navigation simple_navigation.launch.py
```

Or run nodes directly:

```bash
# Terminal 1: Rover serial bridge
ros2 run autonomous_navigation rover_serial_bridge --ros-args -p port:=/dev/ttyAMA0

# Terminal 2: Waypoint follower
ros2 run autonomous_navigation simple_waypoint_follower
```

### Sending Waypoints

Send waypoints via command line (example with 2 waypoints):

```bash
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray "{data: [45.4215, -75.6990, 45.4220, -75.6985]}"
```

Or from Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('waypoint_publisher')
pub = node.create_publisher(Float64MultiArray, '/gps_waypoints', 10)

msg = Float64MultiArray()
msg.data = [
    45.4215, -75.6990,  # Waypoint 1
    45.4220, -75.6985,  # Waypoint 2
    45.4225, -75.6980   # Waypoint 3
]
pub.publish(msg)
```

## Algorithm Overview

### Waypoint Navigation

1. Receives a list of GPS waypoints
2. Calculates distance and bearing to the current target waypoint
3. Computes heading error between current heading and desired bearing
4. Generates velocity commands to minimize heading error while moving forward

### Obstacle Avoidance

1. Monitors LiDAR scan data in a front sector (±60° by default)
2. If obstacle detected within threshold distance:
   - Analyzes left and right sectors
   - Steers towards the side with more clearance
   - Adds ±45° to the desired heading
3. Continues towards waypoint while avoiding obstacles

### Speed Control

- Normal operation: Maximum linear speed
- Sharp turns (>30°): Reduced to 50% speed
- Angular velocity is proportional to heading error (P-controller)

## Configuration

Edit the parameters in `config/waypoint_follower.yaml`:

```yaml
simple_waypoint_follower:
  ros__parameters:
    waypoint_threshold: 2.0        # Waypoint reached distance (m)
    max_linear_speed: 0.5          # Max forward speed (m/s)
    max_angular_speed: 0.8         # Max turning speed (rad/s)
    obstacle_distance: 1.5         # Obstacle detection distance (m)
    obstacle_angle: 60.0           # Front sector angle (degrees)
```

## Topic Remapping

If your topics have different names, update the remappings in the launch file:

```python
remappings=[
    ('/fix', '/your_gps_topic'),
    ('/imu/data', '/your_imu_topic'),
    ('/scan', '/your_lidar_topic'),
]
```

## Limitations

- Uses simple proportional control for heading
- Basic obstacle avoidance (steers left/right, no path planning)
- No dynamic replanning if path is blocked
- Assumes flat terrain (uses haversine distance)
- No recovery behaviors if stuck

## Future Improvements

- Add PID control for smoother navigation
- Implement recovery behaviors
- Add goal preemption capability
- Support for elevation/terrain mapping
- More sophisticated obstacle avoidance
- Waypoint timeout and skipping logic

## Dependencies

- ROS2 (tested with Humble/Foxy)
- sensor_msgs
- geometry_msgs
- std_msgs
- numpy
- pyserial
- tf_transformations

Install Python dependencies:
```bash
pip3 install pyserial numpy tf_transformations
```

## License

MIT
