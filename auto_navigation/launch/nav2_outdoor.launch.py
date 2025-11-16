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

    # --- Nav2 Navigation Stack (manually launched to exclude docking_server) ---
    # Launch individual Nav2 components instead of navigation_launch.py to avoid docking_server
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_yaml],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )
    
    # Collision Monitor
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_yaml]
    )
    
    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_yaml]
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

    # Then Nav2 components (without docking_server)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(collision_monitor)
    ld.add_action(lifecycle_manager_navigation)

    # Finally supporting nodes
    ld.add_action(gps_waypoint_client)
    # ld.add_action(cmdvel_to_json)

    return ld
