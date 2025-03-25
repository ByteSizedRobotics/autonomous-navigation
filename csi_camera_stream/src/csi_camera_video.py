import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import subprocess as sp
import shlex
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')
        
        # Get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'csi_video_stream', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        
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
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        self.camera_info_msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Identity
        self.camera_info_msg.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        
        # Start camera processing
        self.run()
    
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
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    continue
                
                now = self.get_clock().now().to_msg()
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.camera_frame_id
                
                self.camera_info_msg.header.stamp = now
                
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info_msg)
                
                rate.sleep()
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            cap.release()
            process.terminate()
            process.wait()
            self.get_logger().info("Camera node stopped")


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
