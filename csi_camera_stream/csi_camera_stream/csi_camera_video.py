import rclpy
from rclpy.node import Node
import sys
import os
import subprocess as sp
import shlex
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import queue

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')
        
        # Get parameters
        self.declare_parameter('width', 3280)
        self.declare_parameter('height', 2464)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('bitrate', 4000000)  # 4 Mbps bitrate
        self.declare_parameter('quality', 85)       # MJPEG quality

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.bitrate = self.get_parameter('bitrate').value
        self.quality = self.get_parameter('quality').value
        
        # Create publishers with smaller queue sizes for lower latency
        self.image_pub = self.create_publisher(Image, 'csi_video_stream', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        
        # Create bridge for OpenCV to ROS conversion
        self.bridge = CvBridge()
        
        # Frame processing queue for non-blocking operation
        self.frame_queue = queue.Queue(maxsize=3)
        self.should_stop = False
        
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
        
        # Start frame publishing thread
        self.publisher_thread = threading.Thread(target=self.publisher_worker, daemon=True)
        self.publisher_thread.start()
        
        # Start camera processing
        self.run()

    def publisher_worker(self):
        """Separate thread for publishing frames to reduce blocking"""
        while not self.should_stop:
            try:
                # Get frame from queue (blocking with timeout)
                frame_data = self.frame_queue.get(timeout=1.0)
                if frame_data is None:  # Shutdown signal
                    break
                
                frame, timestamp = frame_data
                
                # Convert and publish
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = self.camera_frame_id
                
                self.camera_info_msg.header.stamp = timestamp
                
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info_msg)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Publisher error: {e}")
                break
    
    def run(self):
        # Define the FIFO (named pipe)
        fifo_path = "/tmp/camera_pipe"

        # Ensure the FIFO does not exist before creating
        if os.path.exists(fifo_path):
            os.remove(fifo_path)
        os.mkfifo(fifo_path)

        # Optimized libcamera-vid command for faster streaming
        cmd = (f"libcamera-vid -t 0 "
               f"--width {self.width} --height {self.height} "
               f"--framerate {self.fps} "
               f"--nopreview "       # No preview window
               f"-o {fifo_path}")
        
        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)

        # Give camera time to initialize
        time.sleep(1.0)

        # OpenCV Capture from the named pipe with optimized settings
        cap = cv2.VideoCapture(fifo_path)
        
        # Set buffer size to reduce latency
        
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera pipe")
            process.terminate()
            process.wait()
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
            return
        
        self.get_logger().info("Camera opened successfully using libcamera")
        self.get_logger().info("NATHAN DEBUG1")
        self.get_logger().info("NATHAN DEBUG2")
        
    # Removed frame counting and timing for lightweight operation
        
        try:
            while rclpy.ok() and not self.should_stop:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    continue
                now = self.get_clock().now().to_msg()
                try:
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.frame_queue.put_nowait((frame, now))
                except queue.Full:
                    pass
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
        finally:
            self.should_stop = True
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass
            cap.release()
            process.terminate()
            process.wait()
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
            cv2.destroyAllWindows()
            

def main(args=None):
    rclpy.init(args=args)
    node = CSIVideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure clean shutdown
        if hasattr(node, 'should_stop'):
            node.should_stop = True
        if hasattr(node, 'publisher_thread'):
            node.publisher_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
