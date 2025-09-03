import rclpy
from rclpy.node import Node
import sys
import os
import subprocess as sp
import shlex
import platform
import pathlib
from pathlib import Path
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading

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
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('inference_every_n_frames', 30)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.inference_every_n_frames = self.get_parameter('inference_every_n_frames').value
        
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

        if platform.system() == 'Windows':
            pathlib.PosixPath = pathlib.WindowsPath
        else:
            pathlib.WindowsPath = pathlib.PosixPath
            
        self.model_path = str(Path("/home/adminbyte/ros2_ws/src/autonomous-navigation/csi_camera_stream/csi_camera_stream/best.pt"))

        # Setup debug directory and counter only if debug mode is enabled
        if self.debug_mode:
            self.debug_dir = "/home/adminbyte/ros2_ws/src/autonomous-navigation/csi_camera_stream/csi_camera_stream/debug_images"
            os.makedirs(self.debug_dir, exist_ok=True)
            self.saved_image_counter = 0
            self.get_logger().info("DEBUG MODE ENABLED - Detection images will be saved locally")

        # No queue - just flag-based inference tracking
        self.latest_detections = []
        self.inference_running = False
        self.inference_lock = threading.Lock()
        self.new_detection_available = False

        self.load_model()

        # Start camera processing
        self.run()
    
    def load_model(self):
        try:
            self.get_logger().info(f"Loading YOLOv5 model from {self.model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path, force_reload=True)
            self.get_logger().info("YOLOv5 model loaded successfully")
            self.get_logger().info("DEBUG: Model loading completed")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5 model: {e}")
            exit(1)

    def run_inference_async(self, frame, frame_count):
        """Run inference in a separate thread without queue"""
        def inference_worker():
            try:
                if self.debug_mode:
                    self.get_logger().info(f"INFERENCE: Starting processing frame {frame_count}")
                
                start_time = time.time()
                
                # Resize frame for faster inference
                # small_frame = cv2.resize(frame, (640, 640))
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Run inference
                results = self.model(frame_rgb)
                detections = results.xyxy[0].cpu().numpy()
                
                end_time = time.time()
                inference_time = end_time - start_time
                
                if self.debug_mode:
                    self.get_logger().info(f"INFERENCE: Found {len(detections)} detections for frame {frame_count} in {inference_time:.2f}s")
                
                # Update results with thread safety
                with self.inference_lock:
                    self.latest_detections = detections
                    self.new_detection_available = True
                    self.inference_running = False
                
            except Exception as e:
                self.get_logger().error(f"Error in inference worker: {e}")
                with self.inference_lock:
                    self.inference_running = False
        
        # Start inference thread only if not already running
        with self.inference_lock:
            if not self.inference_running:
                self.inference_running = True
                inference_thread = threading.Thread(target=inference_worker, daemon=True)
                inference_thread.start()
                return True
            else:
                return False

    def run(self):
        fifo_path = "/tmp/camera_pipe"

        if os.path.exists(fifo_path):
            os.remove(fifo_path)
        os.mkfifo(fifo_path)

        cmd = f"libcamera-vid -t 0 --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --inline -o {fifo_path}" #--nopreview
        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)

        # Give camera time to start
        #time.sleep(2)

        cap = cv2.VideoCapture(fifo_path)
        
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera pipe")
            process.terminate()
            process.wait()
            os.remove(fifo_path)
            return
        
        self.get_logger().info("Camera opened successfully using libcamera")
        
        frame_count = 0

        try:
            while rclpy.ok():
                ret, frame = cap.read()
                frame_count += 1

                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    time.sleep(0.1)
                    continue

                now = self.get_clock().now().to_msg()
                
                # Try to start inference on eligible frames (every N frames)
                if frame_count % self.inference_every_n_frames == 0:
                    inference_started = self.run_inference_async(frame.copy(), frame_count)
                    if inference_started:
                        if self.debug_mode:
                            self.get_logger().info(f"Started inference on frame {frame_count}")
                    else:
                        if self.debug_mode:
                            self.get_logger().info(f"Inference busy, skipped frame {frame_count}")
                
                # Draw latest detections on current frame
                detection_found = False
                new_detection = False
                
                with self.inference_lock:
                    current_detections = self.latest_detections.copy()
                    new_detection = self.new_detection_available
                    self.new_detection_available = False  # Reset flag
                
                if len(current_detections) > 0:
                    scale_x = self.width / 640
                    scale_y = self.height / 480
                    
                    for det in current_detections:
                        x1, y1, x2, y2, conf, cls = det
                        
                        if conf > self.confidence_threshold:
                            detection_found = True
                            
                            # Only log when we get NEW detection results
                            if self.debug_mode and new_detection:
                                self.get_logger().info(f"NEW DETECTION: {self.model.names[int(cls)]} with confidence {conf:.2f}")

                            # Scale coordinates back to original frame size
                            x1_scaled = int(x1 * scale_x)
                            y1_scaled = int(y1 * scale_y)
                            x2_scaled = int(x2 * scale_x)
                            y2_scaled = int(y2 * scale_y)
                            
                            label = f"{self.model.names[int(cls)]}: {conf:.2f}"
                            cv2.rectangle(frame, (x1_scaled, y1_scaled), (x2_scaled, y2_scaled), (0, 255, 0), 2)
                            cv2.putText(frame, label, (x1_scaled, y1_scaled - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Save frame only when NEW detections are available
                if self.debug_mode and detection_found and new_detection:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"detection_{timestamp}_{self.saved_image_counter:04d}.jpg"
                    filepath = os.path.join(self.debug_dir, filename)
                    
                    cv2.imwrite(filepath, frame)
                    self.get_logger().info(f"DEBUG: Saved NEW detection frame to {filepath}")
                    self.saved_image_counter += 1
                
                # Convert frame with overlays to ROS message
                inference_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                inference_img_msg.header.stamp = now
                inference_img_msg.header.frame_id = self.camera_frame_id
                
                self.image_pub.publish(inference_img_msg)
        
        finally:
            # Cleanup
            self.get_logger().info("Shutting down...")
            
            cap.release()
            process.terminate()
            process.wait()
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
            self.get_logger().info("Camera stopped and resources cleaned up")

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