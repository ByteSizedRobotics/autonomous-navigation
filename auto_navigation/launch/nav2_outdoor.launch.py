# auto_nav/launch/nav2_outdoor.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auto_nav')
    nav2_param_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Example GPS waypoints (flat list of lat/lon)
    gps_waypoints_list = [
        45.4216, -75.6989,
        45.4218, -75.6985
    ]

    return LaunchDescription([
        # --- (Optional) GPS/IMU → Odom publisher ---
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

        # --- Nav2 Core Servers ---
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

        # --- GPS Waypoint Nav Client (replaces gps_waypoint_client) ---
        Node(
            package='auto_nav',
            executable='gps_waypoint_nav_client',   # updated executable name
            name='gps_waypoint_nav_client',
            output='screen',
            parameters=[{
                'gps_waypoints': gps_waypoints_list,
                'map_origin_lat': 45.4215,          # your map reference latitude
                'map_origin_lon': -75.6990,         # your map reference longitude
                'use_sim_time': False
            }],
            # Delay until Nav2 servers are active (optional enhancement)
            # condition=IfCondition(LaunchConfiguration('start_after_nav2'))
        ),

        # --- Nav2 Lifecycle Manager ---
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
        ),

        # --- (Optional Future) Robot Localization Stack ---
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     output='screen',
        #     parameters=[{
        #         'use_odometry_yaw': True,
        #         'wait_for_datum': False,
        #         'broadcast_utm_transform': True
        #     }]
        # ),
    ])

