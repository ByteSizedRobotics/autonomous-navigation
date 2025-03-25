#!/usr/bin/env python3
import subprocess
import rclpy
import argparse
import signal
import sys

def signal_handler(sig, frame):
    rclpy.loginfo("Caught interrupt signal, shutting down nodes...")
    for process in active_processes:
        if process.poll() is None:  # Check if process is still running
            process.terminate()
    sys.exit(0)

def launch_nodes():
    parser = argparse.ArgumentParser(description='Launch ROS nodes with configurable camera stream')
    parser.add_argument('--stream', type=str, default='standard',
                      choices=['video', 'inference', 'snapshot', 'all'],
                      help='Camera stream type to use (standard, inference, snapshot, or all)')
    
    args = parser.parse_args()
    stream_type = args.stream
    
    rclpy.init_node('launch_nodes', anonymous=True)
    
    global active_processes
    active_processes = []

    try:
        # Start the appropriate camera node based on selection
        if stream_type == 'standard':
            camera_node = subprocess.Popen(["rosrun", "csi_camera_stream", "csi_camera_video.py"])
            active_processes.append(camera_node)
            rclpy.loginfo("CSI Camera Stream started (standard mode)")
        elif stream_type == 'inference':
            camera_node = subprocess.Popen(["rosrun", "csi_camera_stream", "csi_camera_inference.py"])
            active_processes.append(camera_node)
            rclpy.loginfo("CSI Camera Stream with Inference started (inference mode)")
        elif stream_type == 'snapshot':
            camera_node = subprocess.Popen(["rosrun", "csi_camera_stream", "csi_camera_snapshot.py"])
            active_processes.append(camera_node)
            rclpy.loginfo("CSI Camera Snapshot started (snapshot mode)")

        # Start the WebRTC publisher node with appropriate mode parameter
        webrtc_mode = stream_type if stream_type != 'all' else 'all'
        webrtc_node = subprocess.Popen([
            "rosrun", 
            "csi_camera_stream", 
            "webrtc_publisher.py",
            f"_mode:={webrtc_mode}"
        ])
        active_processes.append(webrtc_node)
        rclpy.loginfo(f"WebRTC Publisher started in {webrtc_mode} mode")

        # Register the signal handler for cleanup
        signal.signal(signal.SIGINT, signal_handler)
        
        # Keep script running until ROS shuts down
        rclpy.spin()

    except Exception as e:
        rclpy.logerr(f"Error launching nodes: {e}")
        for process in active_processes:
            if process.poll() is None:  # Check if process is still running
                process.terminate()

if __name__ == "__main__":
    active_processes = []
    launch_nodes()
