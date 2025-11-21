#!/usr/bin/env python3
"""
Simple Autonomous Navigation Node - Fast Avoidance (No GPS)
- Moves forward in a straight line when path is clear
- Uses LiDAR for obstacle avoidance
- Continuously checks obstacles while turning (fast reactive avoidance)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np


class SimpleAutonomousNavigationFastAvoidance(Node):
    def __init__(self):
        super().__init__('simple_autonomous_navigation_fast_avoidance')
        
        # Parameters (reuses waypoint_follower.yaml configuration)
        self.declare_parameter('max_linear_speed', 0.5)    # m/s
        self.declare_parameter('max_angular_speed', 0.8)   # rad/s
        self.declare_parameter('obstacle_distance', 0.5)   # meters
        self.declare_parameter('obstacle_angle', 40.0)     # degrees
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_angle = math.radians(self.get_parameter('obstacle_angle').value)
        
        # State variables
        self.lidar_data = None
        self.last_obstacle_state = False  # Track obstacle detection state changes
        
        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Simple Autonomous Navigation (Fast Avoidance) initialized (No GPS)')
        self.get_logger().info(f'Max speeds: linear={self.max_linear_speed} m/s, angular={self.max_angular_speed} rad/s')
        self.get_logger().info(f'Obstacle detection: {self.obstacle_distance}m at {math.degrees(self.obstacle_angle):.0f}°')
    
    def lidar_callback(self, msg):
        """Store latest LiDAR scan"""
        self.lidar_data = msg
    
    def control_loop(self):
        """Main control loop - move forward unless obstacle detected"""
        if self.lidar_data is None:
            # No LiDAR data yet, stop for safety
            self.publish_velocity(0.0, 0.0)
            return
        
        # Check for obstacles and get avoidance direction
        obstacle_avoidance_angle = self.detect_obstacles()
        
        if obstacle_avoidance_angle is not None:
            # Obstacle detected - turn to avoid
            if not self.last_obstacle_state:  # Only log when obstacle first detected
                self.get_logger().info(f'Obstacle detected! Turning {math.degrees(obstacle_avoidance_angle):.1f}°')
                self.last_obstacle_state = True
            
            # Turn in place (no forward movement) to avoid obstacle
            linear_vel = 0.0
            angular_vel = obstacle_avoidance_angle / abs(obstacle_avoidance_angle) * self.max_angular_speed
            
        else:
            # Path is clear - move forward
            if self.last_obstacle_state:  # Log when obstacle cleared
                self.get_logger().info('Path clear, moving forward')
                self.last_obstacle_state = False
            
            linear_vel = self.max_linear_speed
            angular_vel = 0.0
        
        # Publish velocity command
        self.publish_velocity(linear_vel, angular_vel)
    
    def detect_obstacles(self):
        """
        Detect obstacles using LiDAR and return avoidance angle
        Returns None if no obstacle, otherwise returns angle to steer (radians)
        NOTE: LiDAR is mounted backwards, so we check angles around ±180° (back sector)
        """
        if self.lidar_data is None:
            return None
        
        scan = self.lidar_data
        
        # Check BACK sector for obstacles (which is actually the FRONT of rover due to mounting)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # Divide front sector into left and right
        left_distances = []
        right_distances = []
        front_distances = []
        
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings (including inf)
            if not math.isfinite(range_val) or range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            
            # Check if in BACK sector (±180°), which is the actual front of the rover
            # This checks angles between (π - obstacle_angle) and (π + obstacle_angle)
            angle_from_back = abs(abs(angle) - math.pi)
            
            if angle_from_back < self.obstacle_angle:
                front_distances.append(range_val)
                
                # Determine left vs right relative to rover's actual front
                if angle > 0:  # Positive angles, left side of back = right side of rover front
                    right_distances.append(range_val)
                else:  # Negative angles, right side of back = left side of rover front
                    left_distances.append(range_val)
        
        # Debug: Log front corridor info
        if front_distances:
            min_dist = min(front_distances)
            self.get_logger().info(
                f'Front corridor ({math.degrees(self.obstacle_angle):.0f}° @ ±180°): '
                f'min={min_dist:.2f}m, count={len(front_distances)}, '
                f'threshold={self.obstacle_distance}m',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().warn('No valid readings in front corridor!', throttle_duration_sec=2.0)
        
        # Check if any obstacle is too close in front
        if front_distances and min(front_distances) < self.obstacle_distance:
            # Obstacle in front, decide which way to steer
            
            # Calculate average distance on each side
            avg_left = np.mean(left_distances) if left_distances else float('inf')
            avg_right = np.mean(right_distances) if right_distances else float('inf')
            
            # Steer towards the side with more clearance
            if avg_left > avg_right:
                # Steer left
                return math.radians(45)
            else:
                # Steer right
                return math.radians(-45)
        
        return None
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutonomousNavigationFastAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
