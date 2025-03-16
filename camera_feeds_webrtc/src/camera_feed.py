import rospy
import cv2
import numpy as np
import subprocess as sp
import shlex
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')
        
        # Get parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        
        # Create publishers
        self.image_pub = rospy.Publisher('~image_raw', Image, queue_size=1)
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

    def run(self):
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
            while not rospy.is_shutdown():
                # Capture frame
                ret, frame = cap.read()
                
                if not ret:
                    rospy.logwarn("Failed to capture frame")
                    continue
                
                # Create timestamp
                now = rospy.Time.now()
                
                # Convert frame to ROS image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.camera_frame_id
                
                # Update camera info timestamp
                self.camera_info_msg.header.stamp = now
                
                # Publish image and camera info
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info_msg)
                
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            cap.release()
            process.terminate()
            process.wait()
            rospy.loginfo("Camera node stopped")

if __name__ == '__main__':
    try:
        node = CameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass