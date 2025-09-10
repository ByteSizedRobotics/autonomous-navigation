from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['config/ekf.yaml'],
            output='screen'
        ),

        # GPS → map transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=['config/ekf.yaml'],
            remappings=[('/gps/fix', '/fix'), ('/imu/data', '/imu/data')],
            output='screen'
        ),

        # Nav2
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            parameters=['config/nav2_params.yaml'],
            output='screen'
        ),

        # JSON motor bridge
        Node(
            package='your_package_name',
            executable='cmdvel_to_json',
            name='cmdvel_to_json',
            output='screen'
        )
    ])

