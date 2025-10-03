#!/usr/bin/env python3
"""
Rover Command Center Node

This node acts as the central communication hub for the autonomous rover.
It receives commands from external software and coordinates all rover subsystems.

Author: ByteSizedRobotics
Date: August 29, 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import subprocess
import threading
import json
import time
from enum import Enum

# Standard ROS2 messages
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, NavSatFix


class RoverState(Enum):
    """Rover operational states"""
    IDLE = "idle"
    MANUAL_CONTROL = "manual_control"
    AUTONOMOUS = "autonomous"


class NodeStatus(Enum):
    """Node operational status"""
    OFFLINE = "offline"
    STARTING = "starting"
    RUNNING = "running"
    ERROR = "error"
    STOPPING = "stopping"

class RoverCommandCentre(Node):
    def __init__(self):
        super().__init__('rover_command_centre')

        self.rover_id = 1 # TODO: NATHAN HARDCODED HERE, BUT NEED ENV VAR IN RASPI
        self.num_heartbeats_missed = 0
        
        # Node status tracking
        self.node_status = {
            'gps': NodeStatus.OFFLINE,
            'csi_camera_1': NodeStatus.OFFLINE,
            'obstacle_detection': NodeStatus.OFFLINE,
            'manual_control': NodeStatus.OFFLINE,
            'motor_control': NodeStatus.OFFLINE
        }        
        # System state
        self.rover_state = RoverState.IDLE
        self.last_heartbeat = time.time()
        
        # Publishers for system status
        self.rover_timestamp = self.create_publisher(String, '/timestamp', 10)
        self.node_status_pub = self.create_publisher(String, '/node_status', 10)
        
        # Publisher for forwarding data to rover nodes (e.g., GPS waypoints)
        # self.waypoints_pub = self.create_publisher(String, '/rover_location', 10)

        # Command subscribers for external control
        self.command_sub = self.create_subscription(String, '/command', self.command_callback, 10)
        self.swdata_sub = self.create_subscription(String, '/gps_waypoints', self.swdata_callback, 10)
        self.heartbeat_sub = self.create_subscription(String, '/heartbeat', self.heartbeat_callback, 10)
        
        # Timers
        self.status_timer = self.create_timer(5.0, self.publish_status)
        self.heartbeat_timer = self.create_timer(15.0, self.check_heartbeat)
        # self.node_monitor_timer = self.create_timer(2.0, self.monitor_nodes)
        
        # Node process tracking
        self.node_processes = {}
        
        self.get_logger().info("Rover Command Center initialized and ready")
    
    def command_callback(self, msg):
        """Process commands from external software"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type')
            # command_params = command_data.get('params', {})
            
            self.get_logger().info(f"Received command: {command_type}")
            
            if command_type == 'LaunchRover':
                if (self.verify_rover_id(command_data)):  # verify rover ID before launching
                    self.launch_rover_autonomous()

            elif command_type == 'ManualControl':
                self.launch_manual_control()
                
            elif command_type == 'Stop':
                self.stop_all_rover_nodes()
                
            else:
                self.get_logger().warn(f"Unknown command type: {command_type}")
                
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON command received")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
    
    def heartbeat_callback(self, msg):
        """Update heartbeat from external software"""
        self.last_heartbeat = time.time()
    
    def swdata_callback(self, msg):
        """Process software data and forward to appropriate rover nodes"""
        try:
            data = json.loads(msg.data)
            data_type = data.get('type')
            data_payload = data.get('data', {})
            
            self.get_logger().info(f"Received swdata: {data_type}")
            
            if data_type == 'waypoints':
                # Forward GPS waypoints to autonomous navigation system
                waypoints_msg = String()
                waypoints_msg.data = json.dumps(data_payload)
                # TODO: need to pass waypoints to AUTONAV in format it expects
                # self.waypoints_pub.publish(waypoints_msg)
                # self.get_logger().info(f"Forwarded {len(data_payload.get('waypoints', []))} waypoints to navigation system")
                
            else:
                self.get_logger().warn(f"Unknown data type: {data_type}")
                
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON swdata received")
        except Exception as e:
            self.get_logger().error(f"Error processing swdata: {e}")
    
    def start_node(self, node_name):
        """Start a specific rover node"""
        if node_name not in self.node_status:
            self.get_logger().error(f"Unknown node: {node_name}")
            return False
            
        if self.node_status[node_name] == NodeStatus.RUNNING:
            self.get_logger().info(f"Node {node_name} is already running")
            return True
            
        try:
            self.node_status[node_name] = NodeStatus.STARTING
            
            # Define launch commands for each node
            launch_commands = {
                'gps': 'ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py',
                'csi_camera_1': 'ros2 launch csi_camera_stream csi_camera_stream.launch.py',
                'obstacle_detection': 'ros2 launch obstacle_detection obstacle_detector.launch.py',
                'manual_control': 'ros2 run potrider wasd_control',
                'motor_control': 'ros2 run potrider serial_motor_node --ros-args -p port:=/dev/rover_serial'
            }
            
            if node_name in launch_commands:
                # Start the node process
                process = subprocess.Popen(
                    launch_commands[node_name].split(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                self.node_processes[node_name] = process
                
                # Give the node a moment to start up
                time.sleep(1)
                
                # Check if process is still running (basic health check)
                if process.poll() is None:
                    self.node_status[node_name] = NodeStatus.RUNNING
                    self.get_logger().info(f"Started node: {node_name}")
                    return True
                else:
                    self.node_status[node_name] = NodeStatus.ERROR
                    self.get_logger().error(f"Node {node_name} failed to start - process exited immediately")
                    return False
            else:
                self.get_logger().error(f"No launch command defined for node: {node_name}")
                self.node_status[node_name] = NodeStatus.ERROR
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to start node {node_name}: {e}")
            self.node_status[node_name] = NodeStatus.ERROR
            return False
    
    def stop_node(self, node_name):
        """Stop a specific rover node"""
        if node_name not in self.node_status:
            self.get_logger().error(f"Unknown node: {node_name}")
            return False
            
        try:
            self.node_status[node_name] = NodeStatus.STOPPING
            
            # Terminate the process if it exists
            if node_name in self.node_processes:
                process = self.node_processes[node_name]
                process.terminate()
                process.wait(timeout=5)
                del self.node_processes[node_name]
                
            self.node_status[node_name] = NodeStatus.OFFLINE
            self.get_logger().info(f"Stopped node: {node_name}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop node {node_name}: {e}")
            self.node_status[node_name] = NodeStatus.ERROR
            return False
    
    def stop_all_nodes(self):
        """Stop all rover nodes"""
        self.get_logger().info("Stopping all rover nodes...")
        for node_name in self.node_status.keys():
            self.stop_node(node_name)
    
    def stop_all_movement(self):
        """Send stop commands to all movement systems"""
        self.get_logger().info("Stopping all rover movement")
        
        # Send zero velocity command to motor control
        # This creates a Twist message with all zeros to stop movement
        try:
            # Create a publisher for motor commands if it doesn't exist
            if not hasattr(self, 'motor_command_pub'):
                self.motor_command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            
            # Send stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            # Send multiple stop commands to ensure it's received
            for _ in range(5):
                self.motor_command_pub.publish(stop_cmd)
                time.sleep(0.1)
                
            self.get_logger().info("Stop commands sent to motor control")
            
        except Exception as e:
            self.get_logger().error(f"Failed to send stop commands: {e}")
    
    def launch_rover_autonomous(self):
        """LaunchRover: Start all nodes for autonomous navigation"""
        self.get_logger().info("LaunchRover command received - starting autonomous navigation")
        
        # Set rover to autonomous state
        self.rover_state = RoverState.AUTONOMOUS
        
        # Start required nodes for autonomous navigation
        autonomous_nodes = ['obstacle_detection', 'motor_control', 'gps', 'csi_camera_1', ]
        
        for node_name in autonomous_nodes:
            if self.node_status[node_name] != NodeStatus.RUNNING:
                self.get_logger().info(f"Starting {node_name} for autonomous navigation")
                success = self.start_node(node_name)
                if not success:
                    self.publish_node_status()
                    self.get_logger().error(f"Failed to start {node_name} - autonomous launch incomplete")
                    return False
                time.sleep(1)  # Stagger startup to avoid resource conflicts
        
        # Stop manual control if it's running (autonomous navigation will control movement)
        if self.node_status['manual_control'] == NodeStatus.RUNNING:
            self.get_logger().info("Stopping manual control - autonomous navigation will handle movement")
            self.stop_node('manual_control')
        
        # Publish node status after all startup attempts are complete
        self.publish_node_status()
        
        self.get_logger().info("LaunchRover complete - autonomous navigation active")
        return True

    def verify_rover_id(self, command_data):
        """verify_rover_id: Initial connection handshake with the rover"""
        self.get_logger().info("Launch Rover command received - performing verification of rover ID")

        requested_rover_id = command_data.get('rover_id')
        
        # Convert both to integers for comparison (handles string/int mismatch)
        try:
            requested_id_int = int(requested_rover_id)
            expected_id_int = int(self.rover_id)
            
            if requested_id_int != expected_id_int:
                self.get_logger().error(f"Rover ID mismatch: expected {expected_id_int}, got {requested_id_int}")
                return False
        except (ValueError, TypeError):
            self.get_logger().error(f"Invalid rover ID format: expected integer, got {requested_rover_id}")
            return False

        self.get_logger().info(f"Request for connection to rover {self.rover_id} established successfully")
        return True

    
    def launch_manual_control(self):
        """ManualControl: Stop autonomous navigation and enable manual control"""
        self.get_logger().info("ManualControl command received - switching to manual control mode")
        
        # Set rover to manual control state
        self.rover_state = RoverState.MANUAL_CONTROL
        
        # Stop autonomous navigation node if it exists
        # Note: You'll need to add 'autonomous_navigation' to your node_status dict if you have a dedicated autonomous nav node
        # For now, we'll stop obstacle detection as it's primarily used for autonomous navigation
        
        # Ensure manual control and motor control are running
        manual_control_nodes = ['manual_control', 'motor_control', 'gps', 'obstacle_detection', 'csi_camera_1']
        
        for node_name in manual_control_nodes:
            if self.node_status[node_name] != NodeStatus.RUNNING:
                self.get_logger().info(f"Starting {node_name} for manual control")
                success = self.start_node(node_name)
                if not success:
                    self.get_logger().error(f"Failed to start {node_name} - manual control setup incomplete")
                    return False
                time.sleep(0.5)
        
        # Publish node status after all startup attempts are complete
        self.publish_node_status()
        
        self.get_logger().info("ManualControl complete - ready for software commands")
        return True
    
    def stop_all_rover_nodes(self):
        """Stop: Stop all rover nodes except the communication node"""
        self.get_logger().info("Stop command received - shutting down all rover nodes")
        
        # Set rover to idle state
        self.rover_state = RoverState.IDLE
        
        # List of all nodes to stop (excludes the communication node itself)
        nodes_to_stop = ['gps', 'lidar', 'csi_camera_1', 'obstacle_detection', 'manual_control', 'motor_control']
        
        for node_name in nodes_to_stop:
            if self.node_status[node_name] == NodeStatus.RUNNING:
                self.get_logger().info(f"Stopping {node_name}")
                self.stop_node(node_name)
                time.sleep(0.5)  # Brief delay between stops
        
        # Ensure all movement is stopped
        self.stop_all_movement()
        
        self.get_logger().info("Stop complete - all rover nodes stopped, communication node remains active")
        return True
    
    
    def check_heartbeat(self):
        """Check for heartbeat from external software"""
        time_since_heartbeat = time.time() - self.last_heartbeat
        if time_since_heartbeat > 30:  # 30 seconds timeout
            self.get_logger().warn(f"No heartbeat received for {time_since_heartbeat:.1f} seconds")
            num_heartbeats_missed = getattr(self, 'num_heartbeats_missed', 0) + 1
            self.num_heartbeats_missed = num_heartbeats_missed
            # Optionally trigger emergency protocols
            #count num of times no heartbeat
            if self.rover_state != RoverState.IDLE and num_heartbeats_missed >= 2:
                self.get_logger().info("Missed 2 heartbeats - stopping all rover nodes for safety")
                self.stop_all_rover_nodes()
    
    def publish_status(self):
        """Publish rover state timestamp"""
        # Publish current timestamp
        state_msg = String()
        state_msg.data = str(time.time())
        self.rover_timestamp.publish(state_msg)
    
    def publish_node_status(self):
        """Publish current status of all nodes"""
        status_data = {
            "timestamp": time.time(),
            "nodes": {}
        }
        
        for node_name, status in self.node_status.items():
            status_data["nodes"][node_name] = status.value
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.node_status_pub.publish(status_msg)
        
        self.get_logger().info(f"Published node status: {status_data['nodes']}")


def main(args=None):
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    
    node = RoverCommandCentre()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Rover Command Center shutting down...")
    finally:
        node.stop_all_nodes()  # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
