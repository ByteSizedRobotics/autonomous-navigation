#!/usr/bin/env python3
import os
import argparse
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # CSI Camera 2 node launch action
    camera_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='csi_camera_video_2',
        name='csi_camera_video_2',
        parameters=[{
            'width': 1280,
            'height': 720,
            'fps': 30,
            'jpeg_quality': 70,
            'camera_frame_id': 'csi_camera_2',
            'camera_id': 1  # Second CSI camera (camera 0 is the first)
        }]
    )

    # WebRTC publisher 2 node launch action
    webrtc_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='webRTC_publisher_2',
        name='webRTC_publisher_2',
        parameters=[{
            'video_topic': 'csi_video_stream_2/compressed',
            'webrtc_port': 8767  # Port for second CSI camera
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
