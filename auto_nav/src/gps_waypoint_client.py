import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

import tf2_ros
import tf2_geometry_msgs

class GPSWaypointClient(Node):
    def __init__(self):
        super().__init__('gps_waypoint_client')

        # Action client to Nav2 waypoint follower
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # TF buffer/listener (to transform GPS to map frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Example waypoints (lat, lon) - replace with your own
        self.gps_waypoints = [
            (45.4215, -75.6972),
            (45.4220, -75.6980)
        ]

        self.timer = self.create_timer(5.0, self.send_waypoints)

    def send_waypoints(self):
        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for waypoint follower server...')
            return

        poses = []
        for lat, lon in self.gps_waypoints:
            try:
                # Query navsat_transform for map coordinates
                trans = self.tf_buffer.lookup_transform('map', 'gps', rclpy.time.Time())
                # Create a PoseStamped in gps frame
                gps_pose = PoseStamped()
                gps_pose.header.frame_id = 'gps'
                gps_pose.header.stamp = self.get_clock().now().to_msg()
                gps_pose.pose.position.x = lat
                gps_pose.pose.position.y = lon
                gps_pose.pose.orientation.w = 1.0

                # Transform into map frame
                map_pose = tf2_geometry_msgs.do_transform_pose(gps_pose, trans)
                poses.append(map_pose)

            except Exception as e:
                self.get_logger().warn(f"Could not transform GPS waypoint: {e}")

        if not poses:
            self.get_logger().error("No valid waypoints to send")
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent {len(poses)} waypoints to Nav2")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

