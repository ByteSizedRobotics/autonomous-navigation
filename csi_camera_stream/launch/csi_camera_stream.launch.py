#!/usr/bin/env python3
import os
import argparse
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create an argument for stream type
    stream_type_arg = LaunchConfiguration('stream', default='standard')
    
    # Define the stream type argument
    stream_type_launch_arg = launch.actions.DeclareLaunchArgument(
        'stream',
        default_value='standard',
        choices=['video', 'inference', 'snapshot', 'all'],
        description='Camera stream type to use'
    )

    # Camera node selection based on stream type
    camera_node_map = {
        'standard': 'csi_camera_video',
        'inference': 'csi_camera_inferenc',
        'snapshot': 'csi_camera_snapshot'
    }

    # Select the appropriate camera node
    camera_node_name = camera_node_map.get(stream_type_arg.var_name, 'csi_camera_video')

    # Camera node launch action
    camera_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable=camera_node_name,
        name='csi_camera_node',
        parameters=[{'stream_type': stream_type_arg}]
    )

    # WebRTC publisher node launch action
    webrtc_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='webrtc_publisher_node',
        name='webrtc_publisher',
        parameters=[{'mode': stream_type_arg}]
    )

    # Construct the launch description
    ld = launch.LaunchDescription([
        stream_type_launch_arg,
        camera_node,
        webrtc_node
    ])

    return ld

def main():
    # This allows the launch file to be run directly
    parser = argparse.ArgumentParser(description='Launch ROS2 CSI Camera Nodes')
    parser.add_argument('--stream', type=str, default='standard',
                        choices=['video', 'inference', 'snapshot', 'all'],
                        help='Camera stream type to use')
    args = parser.parse_args()

    # Launch the nodes
    launch.main(launch_description_factory=generate_launch_description)

if __name__ == '__main__':
    main()