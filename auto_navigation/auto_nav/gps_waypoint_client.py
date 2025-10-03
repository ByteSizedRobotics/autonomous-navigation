import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # or a custom message for waypoints

class GPSWaypointClient(Node):
    def __init__(self):
        super().__init__('gps_waypoint_client')

        # Declare waypoints parameter as BYTE_ARRAY
        raw_waypoints = self.declare_parameter(
            'gps_waypoints', []
        ).value  # This must be a flat list of floats

        # Reshape into (lat, lon) pairs
        self.gps_waypoints = [
            (raw_waypoints[i], raw_waypoints[i + 1])
            for i in range(0, len(raw_waypoints), 2)
        ]
        self.get_logger().info(f"Loaded GPS waypoints: {self.gps_waypoints}")

        # Example publisher if you need it
        self.pub = self.create_publisher(String, 'gps_waypoints_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        msg = String()
        msg.data = str(self.gps_waypoints)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

