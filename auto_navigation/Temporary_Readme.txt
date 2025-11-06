We can't extract odometer data from the motors so we'll emulate it using GPS+IMU (Will probably be somewhat inaccurate so don't be surprised if it drifts off by a couple meters)

On top of that, we will be using GPS and IMU for positional navigation.

Five main components:
	
	GPS + IMU → Data will create globe-wide localization
	
	LIDAR → Pretty self explanatory; collision avoidance
	
	Waypoint client → Plot GPS waypoints for navigation planning towards Nav2
	
	Nav2 → Feeds on the given data, outputs velocity commands 
	
	Motor Driver → Will translate Nav2 velocity commands to JSON commands (Slave motor driver board)
	
We'll also need ROS2-localization package; we'll use 'navsat_transform_node' and 'efk_node' to convert IMU and GPS data to Transform Frames (ROS2 coordinate frames over time; uses tf2 library) understandable by both ROS2 and Nav2


====== TO DO ======

	- Test and implement ROS2 -> Nav2

====================

Startup:
	1. Start GPS + IMU drivers → publishing '/fix', '/imu/data' and '/scan' respectively (/imu/raw → raw array data for webapp)
		*Make sure GPS/Rover is plugged in, persistent serial names are set (GPS_serial, rover_serial)
		a. colcon build --packages-select auto_nav
		b. source install/setup.bash
		c. ros2 run auto_nav gps_serial_driver
		d. ros2 run auto_nav rover_serial_bridge
	
	2. Start Waypoint client: ros2 run auto_nav gps_waypoint_client

	2. Launch file: ros2 launch auto_nav bringup_rover.launch.py

	3. Run Nav2 waypoint follower: ros2 run nav2_waypoint_follower waypoint_follower
	
	// WIP
	
	ros2 launch auto_nav nav2_outdoor.launch.py
	
==========
temp:

sudo apt update
sudo apt install -y \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-behavior-tree \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-nav2-regulated-pure-pursuit-controller


