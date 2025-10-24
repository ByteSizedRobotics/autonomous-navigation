# auto_nav/launch/nav2_outdoor.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('auto_nav')
    nav2_param_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # GPS waypoints as flat list of floats
    gps_waypoints_byte_array = [
        45.4215, -75.6972,
        45.4220, -75.6980
    ]

    return LaunchDescription([

        # Node: GPS/IMU → Odom TF
        Node(
            package='auto_nav',
            executable='gps_imu_to_odom',
            name='gps_imu_to_odom',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'gps_frame': 'gps',
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        ),

        # Nav2 core servers
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_param_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_param_file]
        ),
        
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen'
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_param_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_file]
        ),

        # Node: GPS Waypoint Client
        Node(
            package='auto_nav',
            executable='gps_waypoint_client',
            name='gps_waypoint_client',
            output='screen',
            parameters=[{
                'gps_waypoints': gps_waypoints_byte_array,
                'use_sim_time': False
            }]
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        )
    ])

