#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_navigation')
    config_file = os.path.join(pkg_share, 'config', 'waypoint_follower.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Waypoint follower node
    waypoint_follower_node = Node(
        package='autonomous_navigation',
        executable='simple_waypoint_follower',
        name='simple_waypoint_follower',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add remappings here if your topics have different names
            # ('/fix', '/your_gps_topic'),
            # ('/imu/data', '/your_imu_topic'),
            # ('/scan', '/your_lidar_topic'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        waypoint_follower_node,
    ])
