# Quick Start Guide - Autonomous Navigation

## Overview
This package integrates the rover serial bridge with autonomous waypoint navigation. The system reads IMU data from your rover, follows GPS waypoints, and avoids obstacles using LiDAR.

## Quick Start

### 1. Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_navigation
source install/setup.bash
```

### 2. Launch the complete system
```bash
# For GPIO serial (Raspberry Pi pins)
ros2 launch autonomous_navigation autonomous_navigation.launch.py

# For USB serial
ros2 launch autonomous_navigation autonomous_navigation.launch.py serial_port:=/dev/ttyUSB0
```

### 3. Send waypoints
```bash
# Example: Send 3 waypoints
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray \
  "{data: [45.4215, -75.6990, 45.4220, -75.6985, 45.4225, -75.6980]}"
```

## Data Flow

```
Rover Hardware
    ↓ (serial)
rover_serial_bridge
    ↓ /imu/data
simple_waypoint_follower ← /fix (GPS) ← GPS sensor
    ↓                     ← /scan (LiDAR) ← LiDAR sensor
    ↓ /cmd_vel
rover_serial_bridge
    ↓ (serial)
Rover Hardware
```

## Monitoring

### Check if nodes are running
```bash
ros2 node list
# Should show:
# /rover_serial_bridge
# /simple_waypoint_follower
```

### Monitor IMU data
```bash
ros2 topic echo /imu/data
```

### Monitor velocity commands
```bash
ros2 topic echo /cmd_vel
```

### Check waypoint follower status
```bash
ros2 topic echo /rosout | grep simple_waypoint_follower
```

## Configuration

Edit `config/waypoint_follower.yaml` to adjust:
- `waypoint_threshold`: How close to get to waypoints (default: 2.0m)
- `max_linear_speed`: Maximum forward speed (default: 0.5 m/s)
- `max_angular_speed`: Maximum turning speed (default: 0.8 rad/s)
- `obstacle_distance`: Obstacle detection range (default: 1.5m)
- `obstacle_angle`: Front sector for obstacle detection (default: 60°)

## Troubleshooting

### No IMU data
- Check serial connection: `ls -l /dev/ttyAMA0` or `/dev/ttyUSB*`
- Check rover is powered on
- Verify serial port parameter matches your hardware

### Robot not moving
- Check if waypoints were received: `ros2 topic echo /gps_waypoints`
- Verify GPS is publishing: `ros2 topic echo /fix`
- Check cmd_vel is being published: `ros2 topic echo /cmd_vel`
- Verify rover_serial_bridge received the commands (check logs)

### Robot moving erratically
- Reduce speeds in config file
- Check IMU calibration
- Verify LiDAR is working: `ros2 topic echo /scan`

### Stopping at obstacles when shouldn't
- Increase `obstacle_distance` threshold
- Increase `obstacle_angle` to widen detection zone

## Safety Features

1. **Automatic stop**: rover_serial_bridge sends stop commands if no velocity commands received for >100ms
2. **Last waypoint stop**: Robot stops completely when reaching the final waypoint
3. **Obstacle avoidance**: Steers away from obstacles detected by LiDAR

## Topic Reference

| Topic | Type | Purpose |
|-------|------|---------|
| `/fix` | sensor_msgs/NavSatFix | GPS position input |
| `/imu/data` | sensor_msgs/Imu | IMU orientation from rover |
| `/scan` | sensor_msgs/LaserScan | LiDAR scan data |
| `/gps_waypoints` | std_msgs/Float64MultiArray | Waypoint list input |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands to rover |
| `/imu/raw` | std_msgs/Float32MultiArray | Raw IMU values |

## Example Usage

### Single waypoint
```bash
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray \
  "{data: [45.4220, -75.6985]}"
```

### Multiple waypoints (will visit in order)
```bash
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray \
  "{data: [45.4215, -75.6990, 45.4220, -75.6985, 45.4225, -75.6980, 45.4230, -75.6975]}"
```

### Slow and cautious mode
Edit config file:
```yaml
simple_waypoint_follower:
  ros__parameters:
    max_linear_speed: 0.3
    max_angular_speed: 0.5
    obstacle_distance: 2.0
```

### Fast mode (careful!)
```yaml
simple_waypoint_follower:
  ros__parameters:
    max_linear_speed: 0.8
    max_angular_speed: 1.2
    obstacle_distance: 1.0
```
