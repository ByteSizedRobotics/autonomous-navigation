#!/usr/bin/env python3
import os
import argparse
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # USB Camera node launch action
    camera_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='usb_camera_video',
        name='usb_camera_video',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 30,
            'jpeg_quality': 70,
            'camera_frame_id': 'usb_camera',
            'camera_device': '/dev/USB_camera'  # Can be int (e.g., 8) or path (e.g., '/dev/USB_camera')
        }]
    )

    # USB WebRTC publisher node launch action
    webrtc_node = launch_ros.actions.Node(
        package='csi_camera_stream',
        executable='usb_webRTC_publisher',
        name='usb_webRTC_publisher',
        parameters=[{
            'video_topic': 'usb_video_stream/compressed',
            'webrtc_port': 8766  # Port for USB camera
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
