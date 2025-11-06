from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('auto_nav')
    config_dir = os.path.join(pkg_share, 'config')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    ekf_yaml = os.path.join(config_dir, 'ekf.yaml')
    navsat_yaml = os.path.join(config_dir, 'navsat.yaml')
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')

    return LaunchDescription([
        # EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_yaml],
            output='screen'
        ),

        # GPS → map transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_yaml],
            remappings=[
                ('/gps/fix', '/fix'),
                ('/imu/data', '/imu/data'),
                ('/odometry/filtered', '/odometry/filtered')
            ],
            output='screen'
        ),

        # Nav2 bringup (include official launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false'
            }.items(),
        ),

        # JSON motor bridge (cmd_vel → rover serial)
        Node(
            package='auto_nav',
            executable='cmdvel_to_json',
            name='cmdvel_to_json',
            output='screen'
        )
    ])

