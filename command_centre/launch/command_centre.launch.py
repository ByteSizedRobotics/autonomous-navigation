#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import subprocess
import sys


def check_and_install_dependencies():
    """Check if required Python dependencies are installed, and install if missing"""
    required_packages = ['psutil']
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"✓ {package} is already installed")
        except ImportError:
            print(f"✗ {package} not found. Installing...")
            try:
                subprocess.check_call([sys.executable, '-m', 'pip', 'install', package])
                print(f"✓ Successfully installed {package}")
            except subprocess.CalledProcessError as e:
                print(f"✗ Failed to install {package}: {e}")
                print(f"  Please manually install: pip install {package}")


def generate_launch_description():
    # Check and install dependencies before launching
    check_and_install_dependencies()
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
            executable='rover_command_centre',
            name='rover_command_centre',
            output='screen',
            parameters=[{
                'auto_start_nodes': LaunchConfiguration('auto_start_nodes'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            respawn=True,  # Restart if the node crashes
            respawn_delay=2.0,
        ),
        
        # Rosbridge Websocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}]
        )
    ])
