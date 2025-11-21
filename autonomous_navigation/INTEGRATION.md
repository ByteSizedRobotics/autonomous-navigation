# Integration Guide - Autonomous Navigation System

## System Components

This package integrates three main components:
1. **Rover Serial Bridge** - Hardware interface
2. **Simple Waypoint Follower** - Navigation logic
3. **External Sensors** - GPS and LiDAR (provided by other packages)

## How It Works

### Data Flow

```
┌─────────────────┐
│  Wave Rover     │
│   Hardware      │
└────────┬────────┘
         │ Serial (115200 baud)
         │ • IMU data → ROS
         │ • Motor commands ← ROS
         ↓
┌─────────────────────────┐
│ rover_serial_bridge     │
│ • Reads IMU (50Hz)      │
│ • Publishes /imu/data   │
│ • Subscribes /cmd_vel   │
│ • Safety watchdog       │
└────────┬────────────────┘
         │
         ↓ /imu/data
         │
┌─────────────────────────────────┐         ┌──────────────┐
│  simple_waypoint_follower       │ ← /fix  │  GPS Sensor  │
│  • GPS waypoint navigation      │         └──────────────┘
│  • IMU-based heading            │
│  • LiDAR obstacle avoidance     │         ┌──────────────┐
│  • Velocity control (10Hz)      │ ← /scan │ LiDAR Sensor │
└────────┬────────────────────────┘         └──────────────┘
         │
         ↓ /cmd_vel
         │
    (back to rover_serial_bridge)
```

### Navigation Algorithm

1. **Receive Waypoints**
   - Waypoints sent as Float64MultiArray: [lat1, lon1, lat2, lon2, ...]
   - Stored in queue, processed sequentially

2. **Calculate Target Bearing**
   - Haversine formula for distance
   - Great circle bearing calculation
   - Target: current waypoint

3. **Read Current State**
   - GPS position from `/fix`
   - Heading from `/imu/data` (converted from quaternion)
   - Obstacles from `/scan`

4. **Obstacle Detection**
   - Check front sector (±60° default)
   - If obstacle < threshold distance:
     - Compare left vs right clearance
     - Steer toward more open side (±45°)

5. **Compute Velocities**
   - Linear: constant (reduced on sharp turns)
   - Angular: proportional to heading error
   - Published to `/cmd_vel`

6. **Waypoint Completion**
   - When distance < threshold: advance to next waypoint
   - When last waypoint reached: stop completely

### Safety Features

#### Rover Serial Bridge
- **Watchdog Timer**: Sends stop command if no `/cmd_vel` for >100ms
- **Safe Defaults**: Motors stop on startup
- **Error Handling**: Catches serial exceptions, logs errors

#### Waypoint Follower
- **Data Validation**: Won't move without GPS, IMU
- **Final Stop**: Stops at last waypoint (doesn't loop)
- **Speed Reduction**: Slows on sharp turns
- **Obstacle Response**: Steers away from obstacles

## Configuration Options

### Waypoint Follower (config/waypoint_follower.yaml)

```yaml
simple_waypoint_follower:
  ros__parameters:
    waypoint_threshold: 2.0      # Distance to waypoint (m)
    max_linear_speed: 0.5        # Max forward speed (m/s)
    max_angular_speed: 0.8       # Max turn rate (rad/s)
    obstacle_distance: 1.5       # Obstacle threshold (m)
    obstacle_angle: 60.0         # Detection sector (degrees)
```

**Tuning Tips:**
- **waypoint_threshold**: Smaller = more precise, but may circle waypoint
- **max_linear_speed**: Adjust for terrain and robot capabilities
- **max_angular_speed**: Higher = sharper turns, may be unstable
- **obstacle_distance**: Larger = more cautious, may over-react
- **obstacle_angle**: Wider = detects more obstacles

### Rover Serial Bridge (config/rover_serial_bridge.yaml)

```yaml
rover_serial_bridge:
  ros__parameters:
    port: /dev/ttyAMA0  # Serial port
```

**Port Options:**
- `/dev/ttyAMA0`: Raspberry Pi GPIO pins
- `/dev/ttyUSB0`: USB to serial adapter
- Check with: `ls -l /dev/tty*`

## Integration with Other Packages

### GPS Source
You need a node publishing to `/fix` (sensor_msgs/NavSatFix).

Options:
- `nmea_navsat_driver`: For NMEA GPS receivers
- `ublox_gps`: For u-blox GPS modules
- Custom GPS node

Example remapping if your GPS publishes to different topic:
```python
remappings=[('/fix', '/gps/fix')]
```

### LiDAR Source
You need a node publishing to `/scan` (sensor_msgs/LaserScan).

Options:
- `rplidar_ros`: For RPLiDAR (already in your workspace)
- `urg_node`: For Hokuyo LiDAR
- `ldrobot_lidar`: For LD06/LD19 LiDAR

Example remapping:
```python
remappings=[('/scan', '/laser_scan')]
```

## Testing & Validation

### Unit Testing (No Hardware)

1. **Test waypoint calculations:**
```python
# Test distance calculation
from autonomous_navigation.simple_waypoint_follower import SimpleWaypointFollower
lat1, lon1 = 45.4215, -75.6990
lat2, lon2 = 45.4220, -75.6985
dist = SimpleWaypointFollower.haversine_distance(lat1, lon1, lat2, lon2)
print(f"Distance: {dist:.2f}m")  # Should be ~60-70m
```

2. **Test bearing calculation:**
```python
bearing = SimpleWaypointFollower.calculate_bearing(lat1, lon1, lat2, lon2)
print(f"Bearing: {math.degrees(bearing):.1f}°")
```

### Integration Testing (With Hardware)

1. **Test IMU data:**
```bash
ros2 launch autonomous_navigation autonomous_navigation.launch.py
# In another terminal:
ros2 topic echo /imu/data --once
```

2. **Test motor control:**
```bash
# Send test velocity
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
# Robot should move forward slowly for ~100ms then stop
```

3. **Test waypoint following:**
```bash
# Send waypoint 10m north (adjust for your location)
ros2 topic pub --once /gps_waypoints std_msgs/msg/Float64MultiArray \
  "{data: [YOUR_LAT+0.0001, YOUR_LON]}"
```

### Field Testing

1. **Short distance test** (2-5m)
   - Set waypoint_threshold: 1.0
   - Place obstacle in path
   - Verify: reaches waypoint, avoids obstacle

2. **Multi-waypoint test** (create square path)
```python
# 20m square
lat, lon = 45.4215, -75.6990
waypoints = [
    lat + 0.00018, lon,         # North 20m
    lat + 0.00018, lon + 0.00018,  # East 20m
    lat, lon + 0.00018,         # South 20m
    lat, lon                    # West 20m (back to start)
]
```

3. **Long distance test** (50-100m)
   - Monitor continuously
   - Check battery life
   - Verify GPS accuracy

## Troubleshooting

### Common Issues

**Issue: Robot doesn't move**
- Check: GPS publishing? `ros2 topic hz /fix`
- Check: IMU publishing? `ros2 topic hz /imu/data`
- Check: Waypoints received? Look for log message
- Check: Serial connection? `ls -l /dev/ttyAMA0`

**Issue: Robot moves erratically**
- Reduce max speeds in config
- Check IMU calibration (should be stable when stationary)
- Check for magnetic interference near IMU

**Issue: Robot circles waypoint**
- Increase waypoint_threshold
- Check GPS accuracy (HDOP, number of satellites)
- Verify coordinate conversion is correct

**Issue: Robot hits obstacles**
- Decrease obstacle_distance (make it more sensitive)
- Increase obstacle_angle (wider detection)
- Check LiDAR mounting (should see obstacles at ground level)

**Issue: Robot stops randomly**
- Check for LiDAR false positives
- Increase obstacle_distance threshold
- Verify LiDAR max range setting

**Issue: Serial communication fails**
- Check permissions: `sudo usermod -a -G dialout $USER` (logout/login)
- Check port: `ls -l /dev/ttyAMA0` or `ls -l /dev/ttyUSB*`
- Verify baud rate matches rover (115200)

### Debug Commands

```bash
# List all topics
ros2 topic list

# Check topic rates
ros2 topic hz /imu/data
ros2 topic hz /fix
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Monitor logs
ros2 run rqt_console rqt_console

# Check node connections
ros2 node info /simple_waypoint_follower
ros2 node info /rover_serial_bridge

# Record data for analysis
ros2 bag record -a -o test_run
```

## Performance Optimization

### Timing
- IMU: 50 Hz (from rover)
- Control loop: 10 Hz (waypoint_follower)
- LiDAR: depends on sensor (typically 5-10 Hz)
- GPS: 1-10 Hz (depending on receiver)

### CPU Usage
- Expected: 5-15% on Raspberry Pi 4
- If higher: check for busy loops, reduce logging

### Latency
- Target: <100ms from sensor → decision → motor
- Check: `ros2 topic delay`

## Future Enhancements

1. **PID Control**: Replace proportional control with full PID
2. **Dynamic Replanning**: Adjust path if blocked
3. **Goal Preemption**: Accept new waypoints while running
4. **Recovery Behaviors**: Back up if stuck
5. **Terrain Awareness**: Use IMU pitch/roll for slope detection
6. **Speed Scheduling**: Slow for turns, fast on straightaways
7. **Waypoint Editing**: Skip, insert, or modify waypoints on-the-fly

## Additional Resources

- ROS2 Documentation: https://docs.ros.org/
- Wave Rover Manual: (check your product documentation)
- GPS Coordinate Systems: WGS84
- IMU Calibration: Sensor-specific procedures
