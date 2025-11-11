import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import math

class GPSIMUtoOdom(Node):
    def __init__(self):
        super().__init__('gps_imu_to_odom')

        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Latest readings
        self.lat = None
        self.lon = None
        self.yaw = 0.0

        # Origin GPS
        self.origin_set = False
        self.lat0 = 0.0
        self.lon0 = 0.0

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        if not self.origin_set:
            self.lat0 = msg.latitude
            self.lon0 = msg.longitude
            self.origin_set = True

        self.lat = msg.latitude
        self.lon = msg.longitude

        self.publish_odom()

    def imu_callback(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

        self.publish_odom()

    def publish_odom(self):
        if self.lat is None or self.lon is None or not self.origin_set:
            return

        # Convert GPS to local meters using equirectangular approximation
        x = (self.lat - self.lat0) * 111319.5  # meters per degree latitude
        y = (self.lon - self.lon0) * 111319.5 * math.cos(math.radians(self.lat0))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Zero velocities (optional, can integrate later)
        odom.twist.twist = Twist()

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUtoOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

