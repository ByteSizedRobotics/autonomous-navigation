# autonomous-navigation

This repo holds all code for the Autonomous Navigation (AutoNav).

---

# NMEA GPS Driver installation

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
