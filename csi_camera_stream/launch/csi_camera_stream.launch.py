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

    # CAN CHOOSE BETWEEN THESE OPTIONS
    # 'video': 'csi_camera_video',
    # 'inference': 'csi_camera_inference',
    # 'snapshot': 'csi_camera_snapshot'

    camera_node_name = 'csi_camera_video'

    # Camera node launch action
    camera_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable=camera_node_name,
        name=camera_node_name,
    )

    # WebRTC publisher node launch action
    webrtc_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='webRTC_publisher',
        name='webRTC_publisher',
    )

    # Construct the launch description
    ld = launch.LaunchDescription([
        camera_node,
        webrtc_node
    ])

    return ld

def main():
    # Launch the nodes
    launch.main(launch_description_factory=generate_launch_description)

if __name__ == '__main__':
    main()
