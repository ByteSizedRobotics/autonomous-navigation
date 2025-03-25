import rclpy
import cv2
import numpy as np
import subprocess as sp
import shlex
import torch
import os
import time
import threading
import platform
import pathlib
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CSIInferenceNode(Node):
    def __init__(self):
        super().__init__('camera_inference_node')
        
        # Get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Create publishers
        self.raw_image_pub = self.create_publisher(Image, 'detected_pothole_frames_no_bbox', 10)
        self.inference_image_pub = self.create_publisher(Image, 'detected_pothole_frames', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
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
        
    def load_model(self):
        try:
            if platform.system() == 'Windows':
                pathlib.PosixPath = pathlib.WindowsPath
            else:
                pathlib.WindowsPath = pathlib.PosixPath
            
            self.get_logger().info(f"Loading YOLOv5 model from {self.model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
            self.get_logger().info("YOLOv5 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5 model: {e}")
            exit(1)

    def run(self):
        cmd = f"libcamera-vid -t 0 --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --inline -o -"
        process = sp.Popen(shlex.split(cmd), stdout=sp.PIPE)
        cap = cv2.VideoCapture()
        cap.open(f"pipe:{process.stdout.fileno()}", cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera pipe")
            return
        
        self.get_logger().info("Camera opened successfully using libcamera")
        
        rate = self.create_rate(self.fps)
        
        try:
            frames_with_no_detections = 0
            
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    continue
                
                now = self.get_clock().now().to_msg()
                
                try:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = self.model(frame_rgb)
                    detections = results.xyxy[0].cpu().numpy()
                    objects_detected = any(det[4] > self.confidence_threshold for det in detections)
                    
                    if objects_detected:
                        self.get_logger().info(f"Pothole detected with confidence above {self.confidence_threshold}")
                        frames_with_no_detections = 0
                        results.render()
                        rendered_frame = results.ims[0]
                        display_frame = cv2.cvtColor(rendered_frame, cv2.COLOR_RGB2BGR)
                        
                        raw_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        raw_img_msg.header.stamp = now
                        raw_img_msg.header.frame_id = self.camera_frame_id
                        
                        inference_img_msg = self.bridge.cv2_to_imgmsg(display_frame, "bgr8")
                        inference_img_msg.header.stamp = now
                        inference_img_msg.header.frame_id = self.camera_frame_id
                        
                        self.camera_info_msg.header.stamp = now
                        self.raw_image_pub.publish(raw_img_msg)
                        self.inference_image_pub.publish(inference_img_msg)
                        self.camera_info_pub.publish(self.camera_info_msg)
                    else:
                        frames_with_no_detections += 1
                except Exception as e:
                    self.get_logger().error(f"Error during inference: {e}")
                
                rate.sleep()
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            cap.release()
            process.terminate()
            process.wait()
            self.get_logger().info("Camera inference node stopped")

if __name__ == '__main__':
    rclpy.init()
    node = CSIInferenceNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
