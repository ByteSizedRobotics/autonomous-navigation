# autonomous-navigation

This repo holds all code for the Autonomous Navigation (AutoNav).

## V0.1 Obstacle Detection
This version is the first version of the obstacle detection using the Lidar sensor.
The algorithm is simple: it checks if there are ny points within a set radius. If any point is within the radius, an "obstacle" is detected.

### Steps to use
1. Clone the src folder into your ROS2 workspace.
2. In a terminal, cd into the workspace
3. Run ```colcon build —symlink-install```
4. Before running, make sure the lidar is connected and accessible
5. Run sudo `chmod 666 /dev/ttyUSB0`
6. To run the nodes, run `ros2 launch obstacle_detection obstacle_detector.launch.py`
