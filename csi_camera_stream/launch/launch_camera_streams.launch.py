#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
import argparse
import signal
import sys

class NodeLauncher(Node):
    def __init__(self, stream_type):
        super().__init__('node_launcher')
        self.stream_type = stream_type
        self.active_processes = []
        self.get_logger().info(f"Starting nodes with stream type: {stream_type}")

        signal.signal(signal.SIGINT, self.signal_handler)

        self.launch_nodes()

    def signal_handler(self, sig, frame):
        self.get_logger().info("Caught interrupt signal, shutting down nodes...")
        for process in self.active_processes:
            if process.poll() is None:  # Check if process is still running
                process.terminate()
        sys.exit(0)

    def launch_nodes(self):
        try:
            if self.stream_type == 'standard':
                camera_node = subprocess.Popen(["ros2", "run", "csi_camera_stream", "csi_camera_video"])
                self.active_processes.append(camera_node)
                self.get_logger().info("CSI Camera Stream started (standard mode)")
            elif self.stream_type == 'inference':
                camera_node = subprocess.Popen(["ros2", "run", "csi_camera_stream", "csi_camera_inference"])
                self.active_processes.append(camera_node)
                self.get_logger().info("CSI Camera Stream with Inference started (inference mode)")
            elif self.stream_type == 'snapshot':
                camera_node = subprocess.Popen(["ros2", "run", "csi_camera_stream", "csi_camera_snapshot"])
                self.active_processes.append(camera_node)
                self.get_logger().info("CSI Camera Snapshot started (snapshot mode)")

            webrtc_mode = self.stream_type if self.stream_type != 'all' else 'all'
            webrtc_node = subprocess.Popen([
                "ros2", "run", "csi_camera_stream", "webrtc_publisher",
                f"--ros-args", "-p", f"mode:={webrtc_mode}"
            ])
            self.active_processes.append(webrtc_node)
            self.get_logger().info(f"WebRTC Publisher started in {webrtc_mode} mode")

            rclpy.spin(self)

        except Exception as e:
            self.get_logger().error(f"Error launching nodes: {e}")
            for process in self.active_processes:
                if process.poll() is None:
                    process.terminate()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Launch ROS 2 nodes with configurable camera stream')
    parser.add_argument('--stream', type=str, default='standard',
                        choices=['video', 'inference', 'snapshot', 'all'],
                        help='Camera stream type to use (standard, inference, snapshot, or all)')
    args = parser.parse_args()

    node_launcher = NodeLauncher(args.stream)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
