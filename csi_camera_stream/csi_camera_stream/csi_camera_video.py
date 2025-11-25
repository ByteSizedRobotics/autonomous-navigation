import rclpy
from rclpy.node import Node
import sys
import os
import subprocess as sp
import shlex
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')
        
        # Get parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('jpeg_quality', 50)  # JPEG compression quality (1-100) - Lower for better performance
        self.declare_parameter('camera_id', 0)  # Camera index for libcamera

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.camera_id = self.get_parameter('camera_id').value
        
        # Create publishers - Only publish compressed for performance
        # self.image_pub = self.create_publisher(Image, 'csi_video_stream', 1)  # Disabled for performance
        self.compressed_pub = self.create_publisher(CompressedImage, 'csi_video_stream/compressed', 1)
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
        # Define the FIFO (named pipe) with unique name
        fifo_path = f"/tmp/camera_pipe_{self.camera_id}"

        # Ensure the FIFO does not exist before creating
        if os.path.exists(fifo_path):
            os.remove(fifo_path)
        os.mkfifo(fifo_path)

        # Start libcamera-vid process with optimized settings for low latency
        # Using MJPEG codec with quality control and latency optimizations
        cmd = f"libcamera-vid -t 0 --camera {self.camera_id} --width {self.width} --height {self.height} --framerate {self.fps} --codec mjpeg --quality {self.jpeg_quality} --inline --flush -o {fifo_path} --nopreview --denoise off --tuning-file /usr/share/libcamera/ipa/rpi/vc4/imx219_noir.json"
        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)

        # OpenCV Capture from the named pipe
        cap = cv2.VideoCapture(fifo_path)
        
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id} pipe")
            process.terminate()
            process.wait()
            os.remove(fifo_path)
            return
        
        self.get_logger().info(f"CSI Camera {self.camera_id} opened successfully using libcamera")
        self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.fps} FPS")
        self.get_logger().info(f"JPEG Quality: {self.jpeg_quality}")
        
        # Debugging: Check video capture properties
        # self.get_logger().info(f"Capture FPS: {cap.get(cv2.CAP_PROP_FPS)}")
        # self.get_logger().info(f"Frame Width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
        # self.get_logger().info(f"Frame Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        
        frame_count = 0
        consecutive_failures = 0
        max_consecutive_failures = 10
        
        # JPEG compression parameters for additional compression
        jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        
        try:
            while rclpy.ok():
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.0)
                
                ret, frame = cap.read()
                if not ret:
                    consecutive_failures += 1
                    if consecutive_failures <= 3:
                        self.get_logger().warn(f"Failed to capture frame (consecutive failures: {consecutive_failures})")
                    
                    if consecutive_failures >= max_consecutive_failures:
                        self.get_logger().error("Too many consecutive failures. Restarting camera...")
                        cap.release()
                        process.terminate()
                        process.wait()
                        time.sleep(1.0)
                        
                        # Restart libcamera process
                        process = sp.Popen(shlex.split(cmd), stderr=sp.PIPE)
                        cap = cv2.VideoCapture(fifo_path)
                        if not cap.isOpened():
                            self.get_logger().error("Failed to restart camera")
                            break
                        consecutive_failures = 0
                    else:
                        time.sleep(0.01)
                    continue
                
                # Reset failure counter on successful frame
                consecutive_failures = 0
                
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
                
                # Only publish compressed image for WebRTC (raw disabled for performance)
                _, jpeg_buffer = cv2.imencode('.jpg', frame, jpeg_params)
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = now
                compressed_msg.header.frame_id = self.camera_frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = jpeg_buffer.tobytes()
                self.compressed_pub.publish(compressed_msg)
                
                self.camera_info_msg.header.stamp = now
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
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
            cv2.destroyAllWindows()
            self.get_logger().info(f"Captured {frame_count} frames")
            self.get_logger().info(f"CSI Camera {self.camera_id} node stopped")

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