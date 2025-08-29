#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the rover command center'
        ),
        
        DeclareLaunchArgument(
            'auto_start_nodes',
            default_value='false',
            description='Whether to automatically start all rover nodes on launch'
        ),
        
        # Log launch info
        LogInfo(msg="Starting Rover Command Center..."),
        
        # Rover Command Center Node
        Node(
            package='command_centre',
            executable='rover_command_center',
            name='rover_command_center',
            output='screen',
            parameters=[{
                'auto_start_nodes': LaunchConfiguration('auto_start_nodes'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            respawn=True,  # Restart if the node crashes
            respawn_delay=2.0,
        ),
    ])
