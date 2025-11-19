import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import json


class GPSWaypointNavClient(Node):
    def __init__(self):
        super().__init__('gps_waypoint_nav_client')

        # Parameters for reference origin (dummy transform)
        self.declare_parameter('map_origin_lat', 45.4215)
        self.declare_parameter('map_origin_lon', -75.6990)
        self.origin_lat = self.get_parameter('map_origin_lat').value
        self.origin_lon = self.get_parameter('map_origin_lon').value

        # Internal state
        self.gps_waypoints = []          # queue of (lat, lon)
        self.current_goal_active = False # are we currently navigating?
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriber to continuous GPS waypoint stream
        self.subscription = self.create_subscription(
            String,
            '/gps_waypoints',
            self.waypoint_callback,
            10
        )

        self.get_logger().info("GPSWaypointNavClient initialized, listening on /gps_waypoints")

    # --- Receive incoming GPS waypoint ---
    def waypoint_callback(self, msg):
        # Parse JSON format from UI: {"type":"waypoints", "data": {"waypoints": [...]}}
        try:
            data = json.loads(msg.data)
            
            # Handle nested structure from UI
            if 'data' in data and 'waypoints' in data['data']:
                waypoints = data['data']['waypoints']
            elif 'waypoints' in data:
                waypoints = data['waypoints']
            else:
                self.get_logger().warn(f"Invalid message format: missing 'waypoints' field")
                return
            
            if not isinstance(waypoints, list) or len(waypoints) == 0:
                self.get_logger().warn(f"No waypoints in message")
                return
            
            # Add all waypoints to queue
            for wp in waypoints:
                if 'latitude' in wp and 'longitude' in wp:
                    lat = float(wp['latitude'])
                    lon = float(wp['longitude'])
                    self.gps_waypoints.append((lat, lon))
                    self.get_logger().info(f"Queued GPS waypoint {wp.get('id', '?')}: ({lat:.6f}, {lon:.6f})")
                else:
                    self.get_logger().warn(f"Waypoint missing lat/lon: {wp}")
            
            self.get_logger().info(f"Total waypoints in queue: {len(self.gps_waypoints)}")

            # If Nav2 is idle, immediately send first goal
            if not self.current_goal_active and self.gps_waypoints:
                self.send_next_goal()
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse JSON waypoint message: {e}")
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f"Failed to process waypoint data: {e}")

    # --- Send next goal in queue ---
    def send_next_goal(self):
        if not self.gps_waypoints:
            self.get_logger().info("No GPS waypoints in queue.")
            self.current_goal_active = False
            return

        lat, lon = self.gps_waypoints.pop(0)
        x, y = self.dummy_gps_to_map(lat, lon)
        self.get_logger().info(f"Sending Nav2 goal: ({x:.2f}, {y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.current_goal_active = True

    # --- Handle Nav2 goal response ---
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2 server.")
            self.current_goal_active = False
            return

        self.get_logger().info("Goal accepted by Nav2, waiting for completion...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # --- Called when goal completes ---
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Reached GPS waypoint.")
        self.current_goal_active = False
        # Send next waypoint if queue isn't empty
        self.send_next_goal()

    # --- Dummy GPS→map conversion for testing ---
    def dummy_gps_to_map(self, lat, lon):
        dx = (lon - self.origin_lon) * 100000
        dy = (lat - self.origin_lat) * 100000
        return dx, dy


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointNavClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

