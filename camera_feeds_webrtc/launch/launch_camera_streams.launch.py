import subprocess
import rospy

def launch_nodes():
    rospy.init_node('launch_nodes', anonymous=True)

    try:
        # Start the CSI camera node
        camera_node = subprocess.Popen(["rosrun", "your_package", "csi_camera_publisher.py"])
        rospy.loginfo("CSI Camera Publisher started")

        # Start the WebRTC publisher node
        webrtc_node = subprocess.Popen(["rosrun", "your_package", "webrtc_publisher.py"])
        rospy.loginfo("WebRTC Publisher started")

        # Keep script running until ROS shuts down
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down nodes...")
        camera_node.terminate()
        webrtc_node.terminate()

if __name__ == "__main__":
    launch_nodes()
