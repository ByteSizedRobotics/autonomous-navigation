#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # Subscribe to the LaserScan topic (adjust the topic name if necessary)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        # You can set a threshold (in meters) for detecting obstacles
        self.declare_parameter('distance_threshold', 0.25)

    def scan_callback(self, msg: LaserScan):
        threshold = self.get_parameter('distance_threshold').value
        # Check if any scan reading is less than the threshold
        if any(distance < threshold for distance in msg.ranges if distance > 0):
            self.get_logger().warn('Obstacle detected!')
        else:
            self.get_logger().info('Path clear.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

