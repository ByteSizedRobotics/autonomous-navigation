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

    # --- LIDAR Driver TO DO

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

    # --- Nav2 Bringup (no static map, GPS navigation only) ---
    # Disable map_server + amcl before launching nav2
    os.environ["MAP"] = "__NO_MAP__"
    os.environ["AMCL"] = "0"
    os.environ["MAP_SERVER"] = "0"

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
            'params_file': nav2_params_yaml,
            'map': '__NO_MAP__',
            'map_subscribe_transient_local': 'false'
        }
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

    # --- Add all nodes to launch description (correct order) ---
    ld.add_action(gps_serial)
    ld.add_action(rover_serial)

    # NavSat first → EKF second (correct!)
    ld.add_action(navsat_transform)
    ld.add_action(ekf)

    # Then Nav2
    ld.add_action(nav2_launch)

    # Finally supporting nodes
    ld.add_action(gps_waypoint_client)
    ld.add_action(cmdvel_to_json)

    return ld
