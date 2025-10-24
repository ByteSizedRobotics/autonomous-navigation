import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

class USBVideoNode(Node):
    def __init__(self):
        super().__init__('usb_video_node')
        
        # Get parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('camera_device', 8)  # Default to /dev/video0

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.camera_device = self.get_parameter('camera_device').value
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'usb_video_stream', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        
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
        
        # Start camera processing
        self.run()
    
    def run(self):
        # Open USB camera using OpenCV with V4L2 backend for better performance
        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open USB camera device {self.camera_device}")
            return
        
        # IMPORTANT: Set FOURCC to MJPEG FIRST before setting resolution!
        # This is critical for getting 30 FPS at 1280x720
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        
        # Now set camera properties
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Set buffer size to 1 to avoid reading old frames
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Get actual camera properties (may differ from requested)
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        # Get the actual FOURCC to verify MJPEG is being used
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        
        self.get_logger().info(f"USB Camera opened successfully on device {self.camera_device}")
        self.get_logger().info(f"Format: {fourcc_str}")
        self.get_logger().info(f"Resolution: {actual_width}x{actual_height} @ {actual_fps} FPS")
        
        # Update camera info if actual dimensions differ
        if actual_width != self.width or actual_height != self.height:
            self.camera_info_msg.width = actual_width
            self.camera_info_msg.height = actual_height
            fx = actual_width / 2.0
            fy = actual_height / 2.0
            cx = actual_width / 2.0
            cy = actual_height / 2.0
            self.camera_info_msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_info_msg.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        
        frame_count = 0
        consecutive_failures = 0
        max_consecutive_failures = 10
        
        try:
            while rclpy.ok():
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.0)
                
                ret, frame = cap.read()
                if not ret:
                    consecutive_failures += 1
                    self.get_logger().warn(f"Failed to capture frame (consecutive failures: {consecutive_failures})")
                    
                    if consecutive_failures >= max_consecutive_failures:
                        self.get_logger().error("Too many consecutive frame capture failures. Attempting to reconnect...")
                        cap.release()
                        time.sleep(1.0)
                        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
                        if not cap.isOpened():
                            self.get_logger().error("Failed to reconnect to camera")
                            break
                        # Re-apply MJPEG codec after reconnection
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        consecutive_failures = 0
                    else:
                        time.sleep(0.01)  # Short delay to prevent tight looping
                    continue
                
                # Reset failure counter on successful frame
                consecutive_failures = 0
                
                now = self.get_clock().now().to_msg()
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.camera_frame_id
                
                self.camera_info_msg.header.stamp = now
                
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info_msg)
                
                frame_count += 1
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info(f"Captured {frame_count} frames")
            self.get_logger().info("USB Camera node stopped")

def main(args=None):
    rclpy.init(args=args)
    node = USBVideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
