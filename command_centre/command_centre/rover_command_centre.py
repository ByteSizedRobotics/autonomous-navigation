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
import signal
import psutil
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
            'rover': NodeStatus.OFFLINE,
            'csi_camera_1': NodeStatus.OFFLINE,
            'csi_camera_2': NodeStatus.OFFLINE,
            'obstacle_detection': NodeStatus.OFFLINE,
            'manual_control': NodeStatus.OFFLINE,
            'usb_camera': NodeStatus.OFFLINE
            # 'motor_control': NodeStatus.OFFLINE
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
        self.node_status_timer = self.create_timer(10.0, self.publish_node_status)  # Publish node status every 10 seconds
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
            
            # Reset heartbeat tracking when receiving any command
            # Commands indicate the external software is active and connected
            self.last_heartbeat = time.time()
            self.num_heartbeats_missed = 0
            
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
        self.num_heartbeats_missed = 0  # Reset missed count on successful heartbeat
    
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
                'gps': 'ros2 run nmea_navsat_driver nmea_serial_driver',
		        'rover': 'ros2 launch auto_nav nav2_outdoor.launch.py',  # Launch full Nav2 + localization stack
                'csi_camera_1': 'ros2 launch csi_camera_stream csi_camera_stream.launch.py',
                'csi_camera_2': 'ros2 launch csi_camera_stream csi_camera_stream_2.launch.py',
                'obstacle_detection': 'ros2 launch obstacle_detection obstacle_detector.launch.py',
                'manual_control': 'ros2 run potrider wasd_control',
                'usb_camera': 'ros2 launch csi_camera_stream usb_camera_stream.launch.py'
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
                
                # Give the node more time to start up and potentially fail if hardware is missing
                # Hardware-dependent nodes (GPS, IMU) typically fail within 2-3 seconds
                time.sleep(1)
                
                # Check if process has already exited
                exit_code = process.poll()
                if exit_code is not None:
                    # Process has terminated - get output for diagnostics
                    try:
                        stdout, stderr = process.communicate(timeout=1)
                        self.node_status[node_name] = NodeStatus.ERROR
                        self.get_logger().error(f"Node {node_name} failed to start - process exited with code {exit_code}")
                        if stderr:
                            self.get_logger().error(f"Error output: {stderr[:500]}")
                        if stdout:
                            self.get_logger().info(f"Standard output: {stdout[:500]}")
                    except subprocess.TimeoutExpired:
                        self.node_status[node_name] = NodeStatus.ERROR
                        self.get_logger().error(f"Node {node_name} failed - could not read output")
                    return False
                
                # Process is still running - assume success
                # Note: Some nodes may run but fail to connect to hardware
                # They will continue running and log errors, but we mark them as running
                self.node_status[node_name] = NodeStatus.RUNNING
                self.get_logger().info(f"Started node: {node_name}")
                
                # Start a background thread to monitor the node's stderr for errors
                monitor_thread = threading.Thread(
                    target=self._monitor_node_errors,
                    args=(node_name, process),
                    daemon=True
                )
                monitor_thread.start()
                
                return True
            else:
                self.get_logger().error(f"No launch command defined for node: {node_name}")
                self.node_status[node_name] = NodeStatus.ERROR
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to start node {node_name}: {e}")
            self.node_status[node_name] = NodeStatus.ERROR
            return False
    
    def _monitor_node_errors(self, node_name, process):
        """
        Background thread to monitor node stderr for critical errors.
        If critical errors are detected, marks the node as ERROR status.
        """
        critical_errors = [
            'could not open port',
            'permission denied',
            'no such device',
            'device not found',
            'connection refused',
            'failed to connect',
            'cannot open',
            'serial exception'
        ]
        
        try:
            # Read stderr line by line
            for line in iter(process.stderr.readline, ''):
                if not line:
                    break
                    
                line_lower = line.lower()
                
                # Check for critical error indicators
                for error_pattern in critical_errors:
                    if error_pattern in line_lower:
                        self.get_logger().error(f"Critical error detected in {node_name}: {line.strip()}")
                        self.node_status[node_name] = NodeStatus.ERROR
                        return
                
                # Also log all stderr output for debugging
                if line.strip():
                    self.get_logger().warn(f"{node_name} stderr: {line.strip()}")
                    
        except Exception as e:
            self.get_logger().debug(f"Error monitor thread for {node_name} ended: {e}")
    
    def stop_node(self, node_name):
        """Stop a specific rover node and all its child processes"""
        if node_name not in self.node_status:
            self.get_logger().error(f"Unknown node: {node_name}")
            return False
            
        try:
            self.node_status[node_name] = NodeStatus.STOPPING
            
            # Define specific ROS2 node names that should be killed for each node type
            # This helps ensure we catch all related processes
            node_kill_targets = {
                'obstacle_detection': ['rplidar_node', 'obstacle_detector', 'rplidar_composition'],
                'csi_camera_1': ['csi_camera_video'],
                'csi_camera_2': ['csi_camera_video_2'],
                'usb_camera': ['usb_camera_video', 'usb_webRTC_publisher'],
                'gps': ['gps_serial_driver'],
                'rover': ['rover_serial_bridge'],
                'manual_control': ['wasd_control']
            }
            
            # Terminate the tracked process if it exists (this is the launch process)
            if node_name in self.node_processes:
                process = self.node_processes[node_name]
                
                try:
                    # Get the process and all its children using psutil
                    parent = psutil.Process(process.pid)
                    children = parent.children(recursive=True)
                    
                    # Log what we're killing
                    self.get_logger().info(f"Stopping {node_name} (PID: {process.pid}) and {len(children)} child processes")
                    
                    # First, send SIGINT (like Ctrl+C) to give proper shutdown
                    try:
                        parent.send_signal(signal.SIGINT)
                        self.get_logger().info(f"Sent SIGINT to {node_name} launch process")
                        time.sleep(1)  # Give it a moment to shutdown gracefully
                    except Exception as e:
                        self.get_logger().debug(f"SIGINT failed: {e}")
                    
                    # Check if it's still alive
                    if parent.is_running():
                        # Send SIGKILL to all children first
                        for child in children:
                            try:
                                if child.is_running():
                                    self.get_logger().debug(f"Force killing child process: {child.pid} ({child.name()})")
                                    child.kill()
                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                pass
                            except Exception as e:
                                self.get_logger().debug(f"Error killing child {child.pid}: {e}")
                        
                        # Force kill parent
                        try:
                            parent.kill()
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                    
                    # Wait for processes to die
                    gone, alive = psutil.wait_procs(children + [parent], timeout=2)
                    
                    # Log any processes that are still alive
                    if alive:
                        self.get_logger().error(f"Failed to kill {len(alive)} processes!")
                        for p in alive:
                            try:
                                self.get_logger().error(f"  Still alive: PID {p.pid} - {p.name()}")
                            except:
                                pass
                        
                except psutil.NoSuchProcess:
                    self.get_logger().warn(f"Process for {node_name} already terminated")
                except Exception as e:
                    self.get_logger().error(f"Error stopping process tree for {node_name}: {e}")
                    # Fallback to simple kill
                    try:
                        process.kill()
                        process.wait(timeout=2)
                    except:
                        pass
                
                del self.node_processes[node_name]
            
            # Kill by ROS2 node names using pkill (catches orphaned processes)
            if node_name in node_kill_targets:
                for target_node in node_kill_targets[node_name]:
                    try:
                        self.get_logger().info(f"Using pkill for: {target_node}")
                        # Try exact name match
                        subprocess.run(['pkill', '-9', target_node], 
                                     capture_output=True, timeout=2)
                        # Try pattern match in command line
                        subprocess.run(['pkill', '-9', '-f', target_node], 
                                     capture_output=True, timeout=2)
                    except Exception as e:
                        self.get_logger().debug(f"pkill for {target_node} failed: {e}")
            
            # Use psutil to find any remaining processes with matching names
            try:
                if node_name in node_kill_targets:
                    for target_node in node_kill_targets[node_name]:
                        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                            try:
                                cmdline = ' '.join(proc.info['cmdline'] or [])
                                proc_name = proc.info['name'] or ''
                                
                                if target_node in cmdline or target_node in proc_name:
                                    self.get_logger().info(f"Killing remaining process: PID {proc.info['pid']}, Name: {proc_name}")
                                    psutil.Process(proc.info['pid']).kill()
                            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                                pass
            except Exception as e:
                self.get_logger().debug(f"psutil sweep failed: {e}")
            
            # Give a moment for processes to clean up
            time.sleep(0.5)
                
            self.node_status[node_name] = NodeStatus.OFFLINE
            self.get_logger().info(f"Stopped node: {node_name}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop node {node_name}: {e}")
            self.node_status[node_name] = NodeStatus.ERROR
            return False
    
    def emergency_kill_all_ros_nodes(self):
        """
        Emergency function to kill ALL ROS2 nodes related to rover operations.
        This is more aggressive than stop_node and doesn't rely on tracked processes.
        Uses multiple strategies to ensure processes are killed.
        """
        self.get_logger().warn("Emergency kill: Terminating all known rover ROS2 nodes")
        
        # Strategy 1: Kill by exact node executable names
        all_node_executables = [
            'rplidar_node',           # Lidar driver
            'obstacle_detector',      # Obstacle detection
            'csi_camera_inference',   # Camera inference
            'csi_camera_video',       # Camera video
            'csi_camera_video_2',     # Second CSI Camera video
            'usb_camera_video',      # USB Camera video
            'gps_serial_driver',      # GPS driver
            'rover_serial_bridge',      # IMU driver
            'wasd_control',           # Manual control
            'rplidar_composition',    # Alternative lidar node name
        ]
        
        for executable in all_node_executables:
            try:
                # Try multiple kill strategies
                # First: exact name match
                result1 = subprocess.run(['pkill', '-9', executable], 
                                       capture_output=True, timeout=2, text=True)
                # Second: partial match in command line
                result2 = subprocess.run(['pkill', '-9', '-f', executable], 
                                       capture_output=True, timeout=2, text=True)
                
                if result1.returncode == 0 or result2.returncode == 0:
                    self.get_logger().info(f"Killed processes matching: {executable}")
            except Exception as e:
                self.get_logger().debug(f"pkill for {executable} failed: {e}")
        
        # Strategy 2: Kill all ROS2-related launch processes
        # This catches the launch files that spawn the actual nodes
        try:
            self.get_logger().info("Killing ROS2 launch processes...")
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch.*obstacle_detection'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch.*csi_camera'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-9', '-f', 'obstacle_detector.launch.py'], 
                         capture_output=True, timeout=2)
            subprocess.run(['pkill', '-9', '-f', 'csi_camera_stream.launch.py'], 
                         capture_output=True, timeout=2)
        except Exception as e:
            self.get_logger().debug(f"Launch file killing failed: {e}")
        
        # Strategy 3: Use psutil to find and kill any process with our target names
        try:
            killed_count = 0
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    proc_name = proc.info['name'] or ''
                    
                    # Check if any of our target executables are in the command line or process name
                    for target in all_node_executables:
                        if target in cmdline or target in proc_name:
                            self.get_logger().info(f"Killing process: PID {proc.info['pid']}, Name: {proc_name}, CMD: {cmdline[:100]}")
                            proc.kill()
                            killed_count += 1
                            break
                    
                    # Also check for launch files
                    if 'obstacle_detector.launch.py' in cmdline or 'csi_camera_stream.launch.py' in cmdline:
                        self.get_logger().info(f"Killing launch file: PID {proc.info['pid']}")
                        proc.kill()
                        killed_count += 1
                        
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass
            
            if killed_count > 0:
                self.get_logger().info(f"Killed {killed_count} processes using psutil")
            else:
                self.get_logger().warn("No processes found to kill - they may already be stopped")
        except Exception as e:
            self.get_logger().error(f"psutil cleanup failed: {e}")
        
        self.get_logger().info("Emergency kill complete")
    
    def diagnose_running_processes(self):
        """
        Diagnostic function to list all ROS2-related processes currently running.
        Call this to debug what's still running after a stop command.
        """
        self.get_logger().info("=== DIAGNOSTIC: Listing all rover-related processes ===")
        
        keywords = ['rplidar', 'obstacle', 'camera', 'gps', 'rover', 'wasd', 
                   'ros2', 'launch', 'serial_driver']
        
        found_processes = []
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    proc_name = proc.info['name'] or ''
                    
                    # Check if any keyword matches
                    for keyword in keywords:
                        if keyword in cmdline.lower() or keyword in proc_name.lower():
                            found_processes.append({
                                'pid': proc.info['pid'],
                                'name': proc_name,
                                'cmdline': cmdline[:150]  # Truncate for readability
                            })
                            break
                            
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass
            
            if found_processes:
                self.get_logger().warn(f"Found {len(found_processes)} rover-related processes still running:")
                for p in found_processes:
                    self.get_logger().info(f"  PID {p['pid']}: {p['name']} - {p['cmdline']}")
            else:
                self.get_logger().info("No rover-related processes found running")
                
        except Exception as e:
            self.get_logger().error(f"Diagnostic failed: {e}")
        
        self.get_logger().info("=== END DIAGNOSTIC ===")
    
    def stop_all_nodes(self):
        """Stop all rover nodes"""
        self.get_logger().info("Stopping all rover nodes...")
        for node_name in self.node_status.keys():
            self.stop_node(node_name)
        
        # As a safety measure, do an emergency kill to catch any orphaned processes
        self.get_logger().info("Running emergency cleanup to catch any remaining processes...")
        self.emergency_kill_all_ros_nodes()
    
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
        
        # Start required nodes for autonomous navigation'usb_camera',
        autonomous_nodes = ['gps', 'rover', 'obstacle_detection', 'usb_camera', 'csi_camera_1', 'csi_camera_2']
        # autonomous_nodes = ['usb_camera', 'csi_camera_1', 'csi_camera_2']

        
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
        
        # Ensure manual control and motor control are running , 'usb_camera'
        manual_control_nodes = ['manual_control', 'gps', 'obstacle_detection', 'usb_camera', 'csi_camera_1', 'csi_camera_2', 'rover']
        #manual_control_nodes = ['manual_control', 'gps', 'usb_camera', 'csi_camera_1', 'csi_camera_2', 'rover']

        
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
        
        # Run diagnostic to see what's running before we stop
        self.diagnose_running_processes()
        
        # Set rover to idle state
        self.rover_state = RoverState.IDLE
        
        # List of all nodes to stop (excludes the communication node itself) , 'usb_camera'
        nodes_to_stop = ['gps', 'rover', 'csi_camera_1', 'csi_camera_2', 'usb_camera', 'obstacle_detection', 'manual_control']
        
        # Stop all nodes regardless of their current status
        # (they might be in ERROR or STARTING state but still have processes running)
        for node_name in nodes_to_stop:
            self.get_logger().info(f"Stopping {node_name} (current status: {self.node_status[node_name].value})")
            self.stop_node(node_name)
            time.sleep(0.2)  # Brief delay between stops
        
        # Emergency cleanup to catch any orphaned processes (especially lidar)
        self.get_logger().info("Running emergency cleanup to ensure all processes are terminated...")
        self.emergency_kill_all_ros_nodes()
        
        # Give processes time to die
        time.sleep(1)
        
        # Run diagnostic again to see what's still running
        self.diagnose_running_processes()
        
        # Ensure all movement is stopped
        self.stop_all_movement()
        
        # Publish final node status
        self.publish_node_status()
        
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
        # Only publish timestamp if rover is not in IDLE state (emergency stopped)
        # This prevents UI from thinking rover is operational after emergency stop
        if self.rover_state != RoverState.IDLE:
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
