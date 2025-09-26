import rclpy
from rclpy.node import Node
import sys
import os
import subprocess as sp
import shlex
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')
        
        # Get parameters
        self.declare_parameter('width', 820)
        self.declare_parameter('height', 616)
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
        # Define the FIFO (named pipe)
        fifo_path = "/tmp/camera_pipe"

        # Ensure the FIFO does not exist before creating
        if os.path.exists(fifo_path):
            os.remove(fifo_path)
        os.mkfifo(fifo_path)

        # Start libcamera-vid process without preview (remove --nopreview to see camera stream)
        cmd = f"libcamera-vid -t 0 --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --inline -o {fifo_path} --nopreview"
        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)

        # OpenCV Capture from the named pipe
        cap = cv2.VideoCapture(fifo_path)
        
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera pipe")
            process.terminate()
            process.wait()
            os.remove(fifo_path)
            return
        
        self.get_logger().info("Camera opened successfully using libcamera")
        
        # Debugging: Check video capture properties
        # self.get_logger().info(f"Capture FPS: {cap.get(cv2.CAP_PROP_FPS)}")
        # self.get_logger().info(f"Frame Width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
        # self.get_logger().info(f"Frame Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        
        frame_count = 0
        
        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    time.sleep(0.1)  # Short delay to prevent tight looping
                    continue
                
                # Debugging: Add frame count and timestamp to image
                # cv2.putText(frame, 
                #            f"Frame: {frame_count} Time: {time.time() - start_time:.2f}s", 
                #            (10, 30), 
                #            cv2.FONT_HERSHEY_SIMPLEX, 
                #            1, 
                #            (0, 255, 0), 
                #            2)
                
                # Show frame with debugging info
                # cv2.imshow("Video Stream", frame)
                # cv2.waitKey(1)  # Small delay to update window
                
                now = self.get_clock().now().to_msg()
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.camera_frame_id
                
                self.camera_info_msg.header.stamp = now
                
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info_msg)
                
                frame_count += 1
                
                # Optional: Break after certain number of frames for testing
                # if frame_count > 300:
                #     break
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            cap.release()
            process.terminate()
            process.wait()
            cv2.destroyAllWindows()
            self.get_logger().info(f"Captured {frame_count} frames")
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