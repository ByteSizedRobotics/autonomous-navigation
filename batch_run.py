import subprocess

def open_ros2_node(command, workdir):
	subprocess.Popen([
		"gnome-terminal", "--", "bash", "-i", "-c",
		f"cd {workdir} && source install/setup.bash && {command}; exec bash"
	])
	
open_ros2_node("ros2 run nmea_navsat_driver nmea_serial_driver", "/home/adminbyte/Documents/autonomous-navigation/nmea_navsat_driver-ros2")

open_ros2_node("ros2 run potrider serial_motor_node --ros-args -p port:=/dev/ttyAMA0", "/home/adminbyte/Documents/autonomous-navigation/wave_rover_serial_control")

open_ros2_node("ros2 run potrider wasd_control", "/home/adminbyte/Documents/autonomous-navigation/wave_rover_serial_control")
