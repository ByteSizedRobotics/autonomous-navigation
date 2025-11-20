#!/usr/bin/env python3
"""
Simple Autonomous Navigation Node
- Follows GPS waypoints
- Uses IMU for heading
- Uses LiDAR for basic obstacle avoidance
- Does not stop until reaching the last waypoint
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np


class SimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__('simple_waypoint_follower')
        
        # Parameters
        self.declare_parameter('waypoint_threshold', 2.0)  # meters
        self.declare_parameter('max_linear_speed', 0.5)    # m/s
        self.declare_parameter('max_angular_speed', 0.8)   # rad/s
        self.declare_parameter('obstacle_distance', 0.5)   # meters
        self.declare_parameter('obstacle_angle', 60.0)     # degrees
        
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_angle = math.radians(self.get_parameter('obstacle_angle').value)
        
        # State variables
        self.waypoints = []  # List of (lat, lon) tuples
        self.current_waypoint_idx = 0
        self.current_gps = None
        self.current_heading = None  # radians, 0 = North
        self.lidar_data = None
        
        # Subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            Float64MultiArray,
            '/gps_waypoints',
            self.waypoint_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Simple Waypoint Follower initialized')
        self.get_logger().info(f'Waypoint threshold: {self.waypoint_threshold}m')
        self.get_logger().info(f'Max speeds: linear={self.max_linear_speed} m/s, angular={self.max_angular_speed} rad/s')
    
    def waypoint_callback(self, msg):
        """Receive waypoints as pairs of lat/lon"""
        if len(msg.data) % 2 != 0:
            self.get_logger().warn('Invalid waypoint message: must have even number of values')
            return
        
        self.waypoints = []
        for i in range(0, len(msg.data), 2):
            lat, lon = msg.data[i], msg.data[i+1]
            self.waypoints.append((lat, lon))
        
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        for idx, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  Waypoint {idx+1}: ({wp[0]:.6f}, {wp[1]:.6f})')
    
    def gps_callback(self, msg):
        """Update current GPS position"""
        self.current_gps = (msg.latitude, msg.longitude)
    
    def imu_callback(self, msg):
        """Extract heading from IMU quaternion"""
        # Convert quaternion to yaw (heading)
        # Assuming ENU frame: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to 0-2π range where 0 = East (ENU frame)
        # For navigation, we typically want 0 = North, so rotate by π/2
        self.current_heading = (yaw + math.pi/2) % (2 * math.pi)
    
    def lidar_callback(self, msg):
        """Store latest LiDAR scan"""
        self.lidar_data = msg
    
    def control_loop(self):
        """Main control loop"""
        # Check if we have all necessary data
        if self.current_gps is None:
            return
        
        if self.current_heading is None:
            return
        
        if not self.waypoints:
            # No waypoints, stop
            self.publish_velocity(0.0, 0.0)
            return
        
        # Check if we've reached all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!', throttle_duration_sec=5.0)
            self.publish_velocity(0.0, 0.0)
            return
        
        # Get current target waypoint
        target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
        current_lat, current_lon = self.current_gps
        
        # Calculate distance to waypoint
        distance = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)
        
        # Check if we've reached the current waypoint
        if distance < self.waypoint_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}')
            self.current_waypoint_idx += 1
            
            # If that was the last waypoint, stop
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('Final waypoint reached! Stopping.')
                self.publish_velocity(0.0, 0.0)
                return
            
            # Otherwise, continue to next waypoint
            target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
            distance = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)
        
        # Calculate bearing to target
        target_bearing = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        
        # Calculate heading error
        heading_error = self.normalize_angle(target_bearing - self.current_heading)
        
        # Check for obstacles and adjust heading if needed
        obstacle_avoidance_angle = self.detect_obstacles()
        
        if obstacle_avoidance_angle is not None:
            # Obstacle detected, adjust heading
            self.get_logger().info(f'Obstacle detected! Steering {math.degrees(obstacle_avoidance_angle):.1f}°', 
                                   throttle_duration_sec=1.0)
            heading_error += obstacle_avoidance_angle
        
        # Calculate velocities
        linear_vel = self.max_linear_speed
        
        # Reduce speed when turning sharply
        if abs(heading_error) > math.radians(30):
            linear_vel *= 0.5
        
        # Angular velocity proportional to heading error
        angular_vel = np.clip(heading_error * 2.0, -self.max_angular_speed, self.max_angular_speed)
        
        # Log status periodically
        self.get_logger().info(
            f'WP {self.current_waypoint_idx+1}/{len(self.waypoints)} | '
            f'Dist: {distance:.2f}m | '
            f'Heading err: {math.degrees(heading_error):.1f}° | '
            f'Vel: ({linear_vel:.2f}, {angular_vel:.2f})',
            throttle_duration_sec=2.0
        )
        
        # Publish velocity command
        self.publish_velocity(linear_vel, angular_vel)
    
    def detect_obstacles(self):
        """
        Detect obstacles using LiDAR and return avoidance angle
        Returns None if no obstacle, otherwise returns angle to steer (radians)
        """
        if self.lidar_data is None:
            return None
        
        scan = self.lidar_data
        
        # Check front sector for obstacles
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # Divide front sector into left and right
        left_distances = []
        right_distances = []
        front_distances = []
        
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            
            # Check if in front sector
            if abs(angle) < self.obstacle_angle:
                front_distances.append(range_val)
                
                if angle < 0:  # Right side
                    right_distances.append(range_val)
                else:  # Left side
                    left_distances.append(range_val)
        
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
    
    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates in meters"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2 in radians (0 = North)"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        
        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        
        bearing = math.atan2(x, y)
        
        # Normalize to 0-2π
        return (bearing + 2 * math.pi) % (2 * math.pi)
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
