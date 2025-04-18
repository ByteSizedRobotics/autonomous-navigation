import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch configurations
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Get package share directory
    config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_serial_driver.yaml")
    rviz_config_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'rviz', 'rplidar_ros.rviz')

    # Define camera node name
    camera_node_name = 'csi_camera_video'

    # Define nodes
    camera_node = Node(
        package='csi_camera_stream',
        executable=camera_node_name,
        name=camera_node_name,
    )

    webrtc_node = Node(
        package='csi_camera_stream',
        executable='webRTC_publisher',
        name='webRTC_publisher',
    )

    driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Flag to start RViz'),

        # Lidar Driver Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }],
            output='screen'
        ),

        # Obstacle Detection Node
        Node(
            package='obstacle_detection',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[{
                'distance_threshold': 0.35,
                'corridor_width': 0.25,
                'corridor_length': 1.0,
                'forward_direction': 3.14159,
                'enable_debug_output': False
            }],
            output='screen'
        ),

        # Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),

        # Rosbridge Websocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}]
        ),

        # Motor Control Node
        Node(
            package='potrider',
            executable='serial_motor_node',
            name='serial_motor_node',
        ),

        # Camera Nodes
        camera_node,
        webrtc_node,
        driver_node
    ])
