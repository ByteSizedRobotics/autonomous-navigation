import rclpy
from rclpy.node import Node
import cv2
import subprocess as sp
import shlex
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CSIPictureNode(Node):
    def __init__(self):
        super().__init__('csi_picture_node')
        
        # Get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('capture_interval', 5.0)  # 5 seconds interval
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.capture_interval = self.get_parameter('capture_interval').value
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'csi_picture', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
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
        
        # Create timer for image capture
        self.timer = self.create_timer(self.capture_interval, self.capture_and_publish_image)
        self.get_logger().info(f"Starting camera node with {self.capture_interval} second interval")

    def capture_and_publish_image(self):
        # Command to capture a single image using libcamera
        cmd = f"libcamera-still --width {self.width} --height {self.height} -n -o -"
        
        try:
            # Run the command and capture output
            process = sp.Popen(shlex.split(cmd), stdout=sp.PIPE)
            output, _ = process.communicate()
            
            # Decode the image
            nparr = bytearray(output)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if img is None:
                self.get_logger().warn("Failed to capture image")
                return
            
            # Create timestamp
            now = self.get_clock().now().to_msg()
            
            # Convert image to ROS message
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.camera_frame_id
            
            # Update camera info timestamp
            self.camera_info_msg.header.stamp = now
            
            # Publish image and camera info
            self.image_pub.publish(img_msg)
            self.camera_info_pub.publish(self.camera_info_msg)
            
            self.get_logger().info(f"Image captured and published at {now.sec}.{now.nanosec}")
            
        except Exception as e:
            self.get_logger().error(f"Error capturing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CSIPictureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()