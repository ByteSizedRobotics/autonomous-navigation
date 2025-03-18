import rospy
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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class CSIInferenceNode:
    def __init__(self):
        rospy.init_node('camera_inference_node')
        
        # Get parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        self.model_path = rospy.get_param('~model_path', 'best.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        
        # Create publishers
        self.raw_image_pub = rospy.Publisher('detected_potole_frames_no_bbox', Image, queue_size=1)
        self.inference_image_pub = rospy.Publisher('detected_pothole_frames', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('~camera_info', CameraInfo, queue_size=1)
        
        # Create bridge for OpenCV to ROS conversion
        self.bridge = CvBridge()
        
        # Setup camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = self.camera_frame_id
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height
        
        # Simple camera matrix
        fx = self.width / 2.0  # Focal length x
        fy = self.height / 2.0  # Focal length y
        cx = self.width / 2.0   # Principal point x
        cy = self.height / 2.0  # Principal point y
        
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        self.camera_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Identity
        self.camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        
        # Load YOLOv5 model
        self.model = None
        self.load_model()

    def load_model(self):
        """Load the YOLOv5 model"""
        try:
            # Set the appropriate path type based on OS
            if platform.system() == 'Windows':
                pathlib.PosixPath = pathlib.WindowsPath
            else:
                pathlib.WindowsPath = pathlib.PosixPath
                
            rospy.loginfo(f"Loading YOLOv5 model from {self.model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
            rospy.loginfo("YOLOv5 model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv5 model: {e}")
            exit(1)

    def run(self):
        """Main loop for capturing frames and performing inference"""
        # Command to capture frames using libcamera
        cmd = f"libcamera-vid -t 0 --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --inline -o -"
        
        # Open subprocess pipe
        process = sp.Popen(shlex.split(cmd), stdout=sp.PIPE)
        
        # Create video capture from pipe
        cap = cv2.VideoCapture()
        cap.open(f"pipe:{process.stdout.fileno()}", cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            rospy.logerr("Failed to open camera pipe")
            return
            
        rospy.loginfo("Camera opened successfully using libcamera")
        
        rate = rospy.Rate(self.fps)
        
        try:
            last_detection_time = rospy.Time.now()
            frames_with_no_detections = 0
            
            while not rospy.is_shutdown():
                # Capture frame
                ret, frame = cap.read()
                
                if not ret:
                    rospy.logwarn("Failed to capture frame")
                    continue
                
                # Create timestamp
                now = rospy.Time.now()
                
                # Perform inference
                try:
                    # Convert BGR to RGB for YOLOv5
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # Run inference
                    results = self.model(frame_rgb)
                    
                    # Get detections
                    detections = results.xyxy[0].cpu().numpy()
                    objects_detected = any(det[4] > self.confidence_threshold for det in detections)
                    
                    # Only publish if objects are detected above the threshold
                    if objects_detected:
                        # Log detection
                        rospy.loginfo_throttle(1.0, f"Pothole detected with confidence above {self.confidence_threshold}")
                        last_detection_time = now
                        frames_with_no_detections = 0
                        
                        # Render results
                        results.render()
                        rendered_frame = results.ims[0]
                        
                        # Convert back to BGR for OpenCV display
                        display_frame = cv2.cvtColor(rendered_frame, cv2.COLOR_RGB2BGR)
                        
                        # Convert frame to ROS image message for raw stream
                        raw_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        raw_img_msg.header.stamp = now
                        raw_img_msg.header.frame_id = self.camera_frame_id
                        
                        # Convert to ROS image message for inference stream
                        inference_img_msg = self.bridge.cv2_to_imgmsg(display_frame, "bgr8")
                        inference_img_msg.header.stamp = now
                        inference_img_msg.header.frame_id = self.camera_frame_id
                        
                        # Update camera info timestamp
                        self.camera_info_msg.header.stamp = now
                        
                        # Publish raw image, inference image, and camera info
                        self.raw_image_pub.publish(raw_img_msg)
                        # self.inference_image_pub.publish(inference_img_msg) # for now only publish the picture without the detections around it
                        self.camera_info_pub.publish(self.camera_info_msg)
                    else:
                        frames_with_no_detections += 1
                        if frames_with_no_detections % 30 == 0:  # Log every ~1 second at 30fps
                            rospy.logdebug(f"No objects detected for {frames_with_no_detections} frames")
                
                except Exception as e:
                    rospy.logerr(f"Error during inference: {e}")
                
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            cap.release()
            process.terminate()
            process.wait()
            rospy.loginfo("Camera inference node stopped")

if __name__ == '__main__':
    try:
        node = CSIInferenceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass