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

    # Camera node launch action with optimized parameters
    camera_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable=camera_node_name,
        name=camera_node_name,
        parameters=[{
            'width': 1280,
            'height': 720,
            'fps': 30,
            'jpeg_quality': 70,
            'camera_frame_id': 'csi_camera',
            'camera_id': 0  # First CSI camera
        }]
    )

    # WebRTC publisher node launch action with compressed stream
    webrtc_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='webRTC_publisher',
        name='webRTC_publisher',
        parameters=[{
            'video_topic': 'csi_video_stream/compressed',
            'webrtc_port': 8765  # Port for first CSI camera
        }]
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
