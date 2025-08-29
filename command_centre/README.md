# Command Centre Package

This package provides the central communication and control hub for the autonomous rover system.

## Features

- **Centralized Control**: Manages all rover subsystems from a single node
- **Node Management**: Start/stop individual nodes programmatically
- **Health Monitoring**: Monitor health of GPS, LiDAR, camera, and other nodes
- **Emergency Protocols**: Emergency stop and safety systems
- **External Communication**: Receive commands from external software
- **State Management**: Track rover operational states (idle, manual, autonomous, etc.)

## Topics

### Publishers
- `/rover/state` - Current rover operational state
- `/rover/waypoints` - GPS waypoints and navigation data forwarded to rover systems

### Subscribers
- `/rover/command` - Command input from external software (JSON)
- `/rover/swdata` - Software data input (JSON) - waypoints, parameters, manual commands
- `/rover/heartbeat` - Heartbeat from external software
- `/fix` - GPS data (for monitoring)
- `/scan` - LiDAR data (for monitoring)
- `/csi_video_stream` - Camera data (for monitoring)
- `/obstacle_detected` - Obstacle detection status

## Usage

```bash
# Start the command center
ros2 run command_centre rover_command_center

# Or use the launch file
ros2 launch command_centre command_centre.launch.py
```

## Command Format

Commands are sent as JSON strings to `/rover/command`:

```json
{
  "type": "LaunchRover"
}
```

Available commands:
- `LaunchRover` - Start autonomous navigation mode
- `ManualControl` - Switch to manual control mode  
- `Stop` - Stop all rover operations

## Software Data Channel

### GPS Waypoints
Send GPS waypoints for autonomous navigation:
```json
{
  "type": "waypoints",
  "data": {
    "waypoints": [
      {"lat": 40.7128, "lon": -74.0060},
      {"lat": 40.7589, "lon": -73.9851}
    ]
  }
}
```

### Navigation Parameters
Send navigation parameters and behavior settings:
```json
{
  "type": "navigation_params",
  "data": {
    "max_speed": 2.0,
    "obstacle_threshold": 1.5,
    "turning_radius": 0.5
  }
}
```

### Manual Commands
Send manual control commands in manual mode:
```json
{
  "type": "manual_command",
  "data": {
    "linear_x": 1.0,
    "angular_z": 0.5
  }
}
```

## New Software Control Commands

### LaunchRover
Starts autonomous navigation mode with all required nodes:
```json
{"type": "LaunchRover"}
```
- Starts: GPS, LiDAR, Camera, Obstacle Detection, Motor Control
- Stops: Manual Control (if running)
- Sets rover state to AUTONOMOUS
- Enables autonomous navigation to control rover movement

### ManualControl
Switches to manual control mode for software-driven commands:
```json
{"type": "ManualControl"}
```
- Starts: Manual Control, Motor Control, Camera
- Keeps obstacle detection running (for safety)
- Sets rover state to MANUAL_CONTROL
- Enables software to send movement commands

### Stop
Stops all rover operations except communication:
```json
{"type": "Stop"}
```
- Stops all rover nodes: GPS, LiDAR, Camera, Obstacle Detection, Manual Control, Motor Control
- Sends emergency stop commands to motors
- Sets rover state to IDLE
- Communication node remains active for future commands
