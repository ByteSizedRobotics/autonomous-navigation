import rospy
import cv2
import subprocess as sp
import shlex
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CSIPictureNode:
    def __init__(self):
        rospy.init_node('csi_picture_node')
        
        # Get parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        self.capture_interval = rospy.get_param('~capture_interval', 5.0)  # 5 seconds interval
        
        # Create publishers - only for periodic images
        self.image_pub = rospy.Publisher('csi_picture', Image, queue_size=1)
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
        rospy.loginfo(f"Starting camera node with {self.capture_interval} second interval")
        
        rate = rospy.Rate(1.0 / self.capture_interval)  # Rate based on capture interval
        
        try:
            while not rospy.is_shutdown():
                # Capture a single frame
                self.capture_and_publish_image()
                
                # Sleep for the desired interval
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            rospy.loginfo("Camera node stopped")

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
                rospy.logwarn("Failed to capture image")
                return
            
            # Create timestamp
            now = rospy.Time.now()
            
            # Convert image to ROS message
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.camera_frame_id
            
            # Update camera info timestamp
            self.camera_info_msg.header.stamp = now
            
            # Publish image and camera info
            self.image_pub.publish(img_msg)
            self.camera_info_pub.publish(self.camera_info_msg)
            
            rospy.loginfo(f"Image captured and published at {now.to_sec()}")
            
        except Exception as e:
            rospy.logerr(f"Error capturing image: {e}")

if __name__ == '__main__':
    try:
        node = CSIPictureNode()
        node.run()
    except rospy.ROSInterruptException:
        pass