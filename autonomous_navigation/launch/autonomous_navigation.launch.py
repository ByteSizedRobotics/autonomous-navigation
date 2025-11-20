#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_navigation')
    waypoint_config = os.path.join(pkg_share, 'config', 'waypoint_follower.yaml')
    bridge_config = os.path.join(pkg_share, 'config', 'rover_serial_bridge.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for rover communication (ttyAMA0 for GPIO pins, ttyUSB0 for USB)'
    )
    
    # Rover serial bridge node (handles IMU input and cmd_vel output)
    rover_serial_bridge_node = Node(
        package='autonomous_navigation',
        executable='rover_serial_bridge',
        name='rover_serial_bridge',
        output='screen',
        parameters=[
            bridge_config,
            {'port': LaunchConfiguration('serial_port')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # Waypoint follower node (autonomous navigation logic)
    waypoint_follower_node = Node(
        package='autonomous_navigation',
        executable='simple_waypoint_follower',
        name='simple_waypoint_follower',
        output='screen',
        parameters=[
            waypoint_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # IMU data comes from rover_serial_bridge
            # cmd_vel goes to rover_serial_bridge
            # Add GPS topic remapping if needed
            # ('/fix', '/your_gps_topic'),
            # ('/scan', '/your_lidar_topic'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        rover_serial_bridge_node,
        waypoint_follower_node,
    ])
