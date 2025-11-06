from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

//TO DO:
    - Add Lidar driver node to launch file
    
def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('auto_nav')
    config_dir = os.path.join(pkg_dir, 'config')

    ekf_yaml = os.path.join(config_dir, 'ekf.yaml')
    navsat_yaml = os.path.join(config_dir, 'navsat.yaml')
    nav2_params_yaml = os.path.join(config_dir, 'nav2_params.yaml')

    # --- GPS Serial Driver ---
    gps_serial = Node(
        package='auto_nav',
        executable='gps_serial_driver',
        name='gps_serial_driver',
        output='screen'
    )

    # --- IMU / Rover Serial Driver ---
    rover_serial = Node(
        package='auto_nav',
        executable='rover_serial_bridge',
        name='rover_serial_bridge',
        output='screen'
    )

    # --- GPS + IMU to Odometry Fusion ---
    gps_imu_fusion = Node(
        package='auto_nav',
        executable='gps_imu_to_odom',
        name='gps_imu_to_odometry',
        output='screen'
    )
    
    Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'navsat_transform.yaml'),
        ],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/fix', '/fix'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/odometry/gps', '/odometry/gps'),
        ]
    )

    # --- EKF Localization Node ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    # --- NavSat Transform Node ---
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/fix'),
            ('/odometry/filtered', '/odometry/filtered')
        ]
    )

    # --- Nav2 Bringup ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_yaml
        }.items()
    )

    # --- GPS Waypoint Client ---
    gps_waypoint_client = Node(
        package='auto_nav',
        executable='gps_waypoint_client',
        name='gps_waypoint_client',
        output='screen'
    )

    # --- CmdVel → JSON Bridge ---
    cmdvel_to_json = Node(
        package='auto_nav',
        executable='cmdvel_to_json',
        name='cmdvel_to_json',
        output='screen'
    )

    # --- Add all nodes ---
    ld.add_action(gps_serial)
    ld.add_action(rover_serial)
    ld.add_action(gps_imu_fusion)
    ld.add_action(ekf)
    ld.add_action(navsat_transform)
    ld.add_action(nav2_launch)
    ld.add_action(gps_waypoint_client)
    ld.add_action(cmdvel_to_json)

    return ld

