#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        
        # Parameters
        self.declare_parameter('distance_threshold', 0.5)    # Obstacle detection distance (m)
        self.declare_parameter('corridor_width', 1.0)        # Width of detection corridor (m)
        self.declare_parameter('corridor_length', 3.0)       # Length of detection corridor (m)
        self.declare_parameter('forward_direction', 0.0)     # Forward direction in radians (0=+x axis)
        self.declare_parameter('enable_debug_output', True) # Enable detailed debug output
        
    def scan_callback(self, msg: LaserScan):
        # Get parameters
        threshold = self.get_parameter('distance_threshold').value
        corridor_width = self.get_parameter('corridor_width').value
        corridor_length = self.get_parameter('corridor_length').value
        forward_dir = self.get_parameter('forward_direction').value # in radians
        debug = self.get_parameter('enable_debug_output').value
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        obstacle_detected = False
        num_points_in_corridor = 0
        min_obstacle_distance = float('inf')
        
        # Process each point in the laser scan
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if distance <= 0 or math.isnan(distance) or math.isinf(distance):
                continue
                
            # Calculate angle of this point
            scan_angle = angle_min + (i * angle_increment)
            
            # Convert polar to cartesian coordinates (in lidar frame)
            x_lidar = distance * math.cos(scan_angle)
            y_lidar = distance * math.sin(scan_angle)
            
            # Rotate coordinates to align with specified forward direction
            x = x_lidar * math.cos(-forward_dir) - y_lidar * math.sin(-forward_dir)
            y = x_lidar * math.sin(-forward_dir) + y_lidar * math.cos(-forward_dir)
            
            # Check if point is within forward corridor
            in_corridor = (
                x > 0 and                  # Only points ahead in forward direction
                x < corridor_length and    # Within corridor length
                abs(y) < corridor_width/2  # Within corridor width
            )
            
            if in_corridor:
                num_points_in_corridor += 1
                if distance < threshold:
                    obstacle_detected = True
                    min_obstacle_distance = min(min_obstacle_distance, distance)
                    if debug:
                        self.get_logger().info(f'Obstacle at ({x:.2f}, {y:.2f}), distance: {distance:.2f}m')
        
        # Publish obstacle detection results
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_detected_pub.publish(obstacle_msg)
        
        # Publish distance if obstacle detected
        if obstacle_detected:
            distance_msg = Float32()
            distance_msg.data = float(min_obstacle_distance)
            self.obstacle_distance_pub.publish(distance_msg)
            self.get_logger().warn(f'Obstacle detected in forward corridor! Distance: {min_obstacle_distance:.2f}m')
        else:
            self.get_logger().info('Forward corridor clear.')
            
        if debug:
            self.get_logger().debug(f'Points in corridor: {num_points_in_corridor}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()