from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

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

        # Nav2 bringup (as a launch include, not Node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(
                    get_package_share_directory('auto_nav'),
                    'config',
                    'nav2_params.yaml'
                ),
                'use_sim_time': 'false'
            }.items(),
        ),

        # JSON motor bridge
        Node(
            package='auto_nav',
            executable='cmdvel_to_json',
            name='cmdvel_to_json',
            output='screen'
        )
    ])

