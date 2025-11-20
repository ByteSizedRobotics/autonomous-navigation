#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
import serial
import json
import time
import math
import threading
import tf_transformations

class RoverSerialBridge(Node):
    def __init__(self):
        super().__init__('rover_serial_bridge')

        # --- Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.raw_pub = self.create_publisher(Float32MultiArray, '/imu/raw', 10)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            String,
            '/JSON',
            self.listener_callback,
            10
        )
        self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 10)

        # --- Serial Setup ---
        self.declare_parameter('port', '/dev/ttyAMA0') # set to rover_serial for USB, ttyAMA0 for pins
        self.port = self.get_parameter('port').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            self.get_logger().info(f"Opened {self.port} @ 115200 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {self.port}: {e}")
            raise SystemExit

        # --- State ---
        self.last_message_time = time.time()
        self.default_command = '{"T":1,"L":0,"R":0}'

        # Enable IMU streaming
        self.enable_imu_feedback()

        # --- Threads ---
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

        self.timer_thread = threading.Thread(target=self.send_default_command, daemon=True)
        self.timer_thread.start()

    # ---------------- Motor Handling ----------------
    def listener_callback(self, msg):
        """Send motor command received from /JSON topic."""
        try:
            self.ser.write(msg.data.encode() + b'\n')
            self.last_message_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Error sending data: {e}')

    def send_default_command(self):
        """Keep rover safe by sending default stop command if no recent motor msg."""
        while rclpy.ok():
            time.sleep(0.1)
            if time.time() - self.last_message_time > 0.1:  # >100ms gap
                try:
                    self.ser.write(self.default_command.encode() + b'\n')
                except Exception as e:
                    self.get_logger().error(f'Error sending default command: {e}')

    # ---------------- IMU Handling ----------------
    def send_command(self, cmd: dict):
        """Send JSON command to rover."""
        try:
            cmd_str = json.dumps(cmd) + "\r\n"
            self.ser.write(cmd_str.encode())
            time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to send command {cmd}: {e}")

    def enable_imu_feedback(self):
        """Enable IMU feedback stream from rover."""
        self.send_command({"T": 131, "cmd": 1})   # enable feedback
        self.send_command({"T": 142, "cmd": 20})  # 20 ms interval (50 Hz)
    
    def cmdvel_callback(self, msg):
        # Convert Twist (linear.x, angular.z) to differential drive (left, right)
        # linear.x = forward speed, angular.z = rotation speed
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive conversion
        # For a rover: left = linear - angular, right = linear + angular
        # Adjust the angular scaling factor if needed (0.5 is typical for wheeled robots)
        wheel_base = 0.5  # Adjust based on your rover's wheel separation
        left = linear - (angular * wheel_base)
        right = linear + (angular * wheel_base)
        
        # Convert to rover's expected JSON format
        cmd = {
            "T": 1,
            "L": int(left * 100),   # Scale to rover's expected range (adjust multiplier as needed)
            "R": int(right * 100)
        }
        json_data = json.dumps(cmd)
        try:
            self.ser.write((json_data + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent: {json_data}")
            self.last_message_time = time.time()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        """Continuously read from rover serial (IMU + any responses)."""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)

                    if "T" in data and data["T"] == 1001:
                        self.publish_imu(data)
                except json.JSONDecodeError:
                    self.get_logger().debug(f"Non-JSON line: {line}")

            except Exception as e:
                self.get_logger().error(f"Error reading data: {e}")

    def publish_imu(self, data: dict):
        """Publish IMU data to ROS topics."""
        roll = float(data.get("r", 0.0))
        pitch = float(data.get("p", 0.0))
        yaw = float(data.get("y", 0.0))
        temp = float(data.get("temp", 0.0))
        v = float(data.get("v", 0.0))

        # Convert Euler to quaternion
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_pub.publish(imu_msg)

        raw_msg = Float32MultiArray()
        raw_msg.data = [roll, pitch, yaw, temp, v]
        self.raw_pub.publish(raw_msg)

    # ---------------- Shutdown ----------------
    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
