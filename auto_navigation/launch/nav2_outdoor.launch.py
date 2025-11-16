from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('auto_nav')
    config_dir = os.path.join(pkg_dir, 'config')

    ekf_yaml = os.path.join(config_dir, 'ekf.yaml')
    navsat_yaml = os.path.join(config_dir, 'navsat.yaml')
    nav2_params_yaml = os.path.join(config_dir, 'nav2_params.yaml')

    # --- TF Transforms (must launch first) ---
    wave_rider_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'wave_rider_tf.launch.py')
        )
    )

    # --- LIDAR Driver TODO: NATHAN

    # --- GPS Serial Driver --- TODO: should be NMEA GPS one?
    # gps_serial = Node(
    #     package='auto_nav',
    #     executable='gps_serial_driver',
    #     name='gps_serial_driver',
    #     output='screen'
    # )

    # --- IMU / Rover Serial Driver ---
    rover_serial = Node(
        package='auto_nav',
        executable='rover_serial_bridge',
        name='rover_serial_bridge',
        output='screen'
    )

    # --- NavSat Transform ---
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml],
        remappings=[
            ('imu/data', '/imu/data'),
            ('fix', '/fix'),
            ('odometry/filtered', '/odometry/filtered'),
            ('gps/odom', '/gps/odom')   # IMPORTANT: Correct GPS odom output
        ]
    )

    # --- EKF Localization (fuses IMU + GPS Odom) ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[
            ('/gps/odom', '/gps/odom'),
            ('/imu/data', '/imu/data')
        ]
    )

    # --- Nav2 Navigation Stack (no map_server or amcl) ---
    # Use navigation_launch.py instead of bringup_launch.py to avoid map_server/amcl
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_yaml,
            'autostart': 'true'
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
    # cmdvel_to_json = Node(
    #     package='auto_nav',
    #     executable='cmdvel_to_json',
    #     name='cmdvel_to_json',
    #     output='screen'
    # )

    # --- Add all nodes to launch description (correct order) ---
    # TF transforms FIRST
    ld.add_action(wave_rider_tf_launch)
    
    # ld.add_action(gps_serial)  # TODO: Uncomment when GPS driver is implemented
    ld.add_action(rover_serial)

    # NavSat first → EKF second (correct!)
    ld.add_action(navsat_transform)
    ld.add_action(ekf)

    # Then Nav2
    ld.add_action(nav2_launch)

    # Finally supporting nodes
    ld.add_action(gps_waypoint_client)
    # ld.add_action(cmdvel_to_json)

    return ld
