import rclpy
from rclpy.node import Node
import sys
import os
import subprocess as sp
import shlex
import platform
import pathlib
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

sys.path.insert(0, '/home/adminbyte/venv/lib/python3.12/site-packages')
import numpy as np
import torch

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')
        
        # Get parameters
        self.declare_parameter('width', 1640)
        self.declare_parameter('height', 1232)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'csi_video_stream', 1)
        
        # Create bridge for OpenCV to ROS conversion
        self.bridge = CvBridge()
        
        # Setup camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = self.camera_frame_id
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height
        
        # Simple camera matrix
        fx = self.width / 2.0
        fy = self.height / 2.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.camera_info_msg.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        
        # Load YOLOv5 model
        self.model = None
        self.load_model()

        # Start camera processing
        self.run()
    
    def load_model(self):
        try:
            if platform.system() == 'Windows':
                pathlib.PosixPath = pathlib.WindowsPath
            else:
                pathlib.WindowsPath = pathlib.PosixPath
            
            self.get_logger().info(f"Loading YOLOv5 model from {self.model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', force_reload=True)
            self.get_logger().info("YOLOv5 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5 model: {e}")
            exit(1)

    def run(self):
        fifo_path = "/tmp/camera_pipe"

        if os.path.exists(fifo_path):
            os.remove(fifo_path)
        os.mkfifo(fifo_path)

        cmd = f"libcamera-vid -t 0 --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --inline -o {fifo_path} --nopreview"
        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)

        cap = cv2.VideoCapture(fifo_path)
        
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera pipe")
            process.terminate()
            process.wait()
            os.remove(fifo_path)
            return
        
        self.get_logger().info("Camera opened successfully using libcamera")
        rate = self.create_rate(self.fps)

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                time.sleep(0.1)
                continue

            now = self.get_clock().now().to_msg()
        
        try:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.model(frame_rgb)
            detections = results.xyxy[0].cpu().numpy()

            # Draw bounding boxes on frame
            for det in detections:
                x1, y1, x2, y2, conf, cls = det
                if conf > self.confidence_threshold:
                    label = f"{self.model.names[int(cls)]}: {conf:.2f}"
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Convert frame with overlays to ROS message
            inference_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            inference_img_msg.header.stamp = now
            inference_img_msg.header.frame_id = self.camera_frame_id
            
            # raw_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # raw_img_msg.header.stamp = now
            # raw_img_msg.header.frame_id = self.camera_frame_id
            
            # self.camera_info_msg.header.stamp = now
            
            # self.raw_image_pub.publish(raw_img_msg)
            self.image_pub.publish(inference_img_msg)
            # self.camera_info_pub.publish(self.camera_info_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")
        
        rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = CSIVideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
