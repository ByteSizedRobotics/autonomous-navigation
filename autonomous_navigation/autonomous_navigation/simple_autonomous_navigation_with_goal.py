#!/usr/bin/env python3
"""
Simple Autonomous Navigation Node with Goal Direction (Bug Algorithm)
- Uses IMU to maintain a global heading (Goal).
- Uses LiDAR (mounted backwards) to detect obstacles.
- Implements a "Bug 0" inspired algorithm:
  1. GO_TO_GOAL: Move towards the target heading.
  2. FOLLOW_WALL: If blocked, follow the obstacle boundary (Right-hand rule) until the path to the goal is clear.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import math
import numpy as np
import time

class SimpleAutonomousNavigationWithGoal(Node):
    def __init__(self):
        super().__init__('simple_autonomous_navigation_with_goal')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.3)    # m/s (slower for precision)
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s
        self.declare_parameter('obstacle_distance', 0.6)   # meters (desired wall distance)
        self.declare_parameter('stop_distance', 0.5)       # meters (emergency stop/turn distance)
        self.declare_parameter('heading_tolerance', 10.0)  # degrees
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wall_distance = self.get_parameter('obstacle_distance').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.heading_tolerance = math.radians(self.get_parameter('heading_tolerance').value)
        
        # State variables
        self.lidar_data = None
        self.current_heading = None
        self.goal_heading = None
        self.state = 'INIT'  # INIT, GO_TO_GOAL, FOLLOW_WALL
        
        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Autonomous Navigation (Bug Algorithm) Initialized')
        self.get_logger().info('Waiting for IMU to set goal heading...')

    def imu_callback(self, msg):
        """Extract heading from IMU quaternion"""
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Normalize to 0-2pi
        self.current_heading = (yaw + 2 * math.pi) % (2 * math.pi)
        
        if self.goal_heading is None:
            self.goal_heading = self.current_heading
            self.state = 'GO_TO_GOAL'
            self.get_logger().info(f'Goal Heading Set: {math.degrees(self.goal_heading):.1f}°')

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def get_scan_range(self, angle_deg, width_deg=20):
        """
        Get min distance in a sector centered at angle_deg (Robot Frame).
        0 = Front, 90 = Left, -90 = Right.
        Handles the 180-degree backwards mounting of the LiDAR.
        """
        if self.lidar_data is None:
            return float('inf')
            
        # Robot Frame to LiDAR Frame mapping:
        # Robot Front (0) -> LiDAR Back (180/-180)
        # Robot Left (90) -> LiDAR Right (-90)
        # Robot Right (-90) -> LiDAR Left (90)
        # Formula: lidar_angle = robot_angle + pi
        
        target_angle_rad = math.radians(angle_deg)
        width_rad = math.radians(width_deg)
        
        min_r = float('inf')
        
        angle_min = self.lidar_data.angle_min
        angle_inc = self.lidar_data.angle_increment
        
        # Iterate through all ranges (simple and robust)
        for i, r in enumerate(self.lidar_data.ranges):
            if not math.isfinite(r) or r < self.lidar_data.range_min or r > self.lidar_data.range_max:
                continue
                
            # Calculate angle of this ray in LiDAR frame
            lidar_angle = angle_min + i * angle_inc
            
            # Convert to Robot Frame: robot = lidar - pi
            robot_angle = self.normalize_angle(lidar_angle - math.pi)
            
            # Check if within sector
            if abs(self.normalize_angle(robot_angle - target_angle_rad)) < (width_rad / 2.0):
                if r < min_r:
                    min_r = r
                    
        return min_r

    def control_loop(self):
        if self.state == 'INIT' or self.lidar_data is None or self.current_heading is None:
            self.publish_velocity(0.0, 0.0)
            return

        # 1. Perception
        front_dist = self.get_scan_range(0, 30)      # Front sector
        right_dist = self.get_scan_range(-90, 30)    # Right sector (for wall following)
        right_front_dist = self.get_scan_range(-45, 30) # Right-Front (for corner anticipation)
        
        # Calculate heading error to goal
        heading_error = self.normalize_angle(self.goal_heading - self.current_heading)
        
        # Check if path to goal is clear (look in direction of goal)
        # We only check if the goal is roughly in front or to the left (since we follow right wall)
        # If goal is to the right (through the wall), we can't go there yet.
        goal_is_reachable = False
        if abs(heading_error) < math.radians(90): # Goal is in front hemisphere
            dist_to_goal_dir = self.get_scan_range(math.degrees(heading_error), 20)
            if dist_to_goal_dir > self.stop_distance * 2.0:
                goal_is_reachable = True

        # 2. State Machine
        linear_vel = 0.0
        angular_vel = 0.0
        
        if self.state == 'GO_TO_GOAL':
            if front_dist < self.stop_distance:
                self.state = 'FOLLOW_WALL'
                self.get_logger().info("Obstacle detected! Switching to Wall Following (Right)")
            else:
                # Proportional controller for heading
                angular_vel = heading_error * 1.5
                linear_vel = self.max_linear_speed
                
                # Slow down if turning
                if abs(angular_vel) > 0.5:
                    linear_vel *= 0.5

        elif self.state == 'FOLLOW_WALL':
            # Exit condition: Path to goal is clear AND we are aligned enough to not hit the wall immediately
            if goal_is_reachable and abs(heading_error) < math.radians(45):
                self.state = 'GO_TO_GOAL'
                self.get_logger().info("Path to goal clear! Resuming Goal Seek")
            else:
                # Wall Following Logic (Keep obstacle on Right)
                # We want to maintain 'wall_distance' from the obstacle on the right
                
                if front_dist < self.stop_distance:
                    # Blocked in front (corner or new obstacle) -> Turn Left sharply
                    linear_vel = 0.0
                    angular_vel = 0.8 # Rotate Left
                else:
                    # P-Controller for Wall Following
                    # Error = Desired - Actual
                    # If Right is too close (dist < desired), Error > 0 -> Turn Left (+z)
                    # If Right is too far (dist > desired), Error < 0 -> Turn Right (-z)
                    
                    # Use min of right and right_front to handle corners better
                    current_wall_dist = min(right_dist, right_front_dist)
                    
                    # If we lost the wall (too far), we need to find it (Turn Right)
                    # But limit the search radius
                    if current_wall_dist > self.wall_distance * 2.0:
                        # Wall lost, curve right gently to find it, but move forward
                        error = -0.5 # Artificial error to induce right turn
                    else:
                        error = self.wall_distance - current_wall_dist
                    
                    angular_vel = error * 2.0
                    linear_vel = self.max_linear_speed * 0.6 # Slower while wall following
                    
        # Clamp velocities
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
        
        self.publish_velocity(linear_vel, angular_vel)

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutonomousNavigationWithGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
