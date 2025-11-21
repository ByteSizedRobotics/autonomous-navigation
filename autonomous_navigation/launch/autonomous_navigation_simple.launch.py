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
    
    # Rover serial bridge node
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
    
    # Simple autonomous navigation (deliberate turns)
    autonomous_nav_node = Node(
        package='autonomous_navigation',
        executable='autonomous_navigation',
        name='autonomous_navigation',
        output='screen',
        parameters=[
            waypoint_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        rover_serial_bridge_node,
        autonomous_nav_node,
    ])
