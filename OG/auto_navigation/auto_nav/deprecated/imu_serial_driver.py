#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import serial
import json
import time
import math
import tf_transformations


class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.raw_pub = self.create_publisher(Float32MultiArray, '/imu/raw', 10)

        # Open serial port (adjust device & baudrate as needed)
        try:
            self.ser = serial.Serial('/dev/rover_serial', 115200, timeout=1)
            self.get_logger().info("Opened /dev/rover_serial @ 115200 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open /dev/rover_serial: {e}")
            raise SystemExit

        # Enable continuous IMU feedback stream
        self.enable_imu_feedback()

        # Timer to read IMU JSON data
        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz poll

    def send_command(self, cmd: dict):
        """Helper to send JSON command over serial."""
        try:
            cmd_str = json.dumps(cmd) + "\r\n"
            self.ser.write(cmd_str.encode())
            self.get_logger().info(f"Sent command: {cmd_str.strip()}")
            time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to send command {cmd}: {e}")

    def enable_imu_feedback(self):
        """Enable continuous chassis feedback (includes IMU)."""
        self.send_command({"T": 131, "cmd": 1})   # enable feedback
        self.send_command({"T": 142, "cmd": 20})  # 20 ms interval (50 Hz)

    def read_serial(self):
        """Read JSON packets from rover and publish IMU data."""
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return

            try:
                data = json.loads(line)

                # Only process IMU-related packets (T=1001 per your example)
                if "T" in data and data["T"] == 1001:
                    roll = float(data.get("r", 0.0))
                    pitch = float(data.get("p", 0.0))
                    yaw = float(data.get("y", 0.0))
                    temp = float(data.get("temp", 0.0))
                    v = float(data.get("v", 0.0))

                    # --- Publish as sensor_msgs/Imu ---
                    roll_rad = math.radians(roll)
                    pitch_rad = math.radians(pitch)
                    yaw_rad = math.radians(yaw)

                    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
                        roll_rad, pitch_rad, yaw_rad
                    )

                    imu_msg = Imu()
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    # Leave angular velocity & linear acceleration at 0
                    self.imu_pub.publish(imu_msg)

                    # --- Publish raw fields ---
                    raw_msg = Float32MultiArray()
                    raw_msg.data = [roll, pitch, yaw, temp, v]
                    self.raw_pub.publish(raw_msg)

                    self.get_logger().debug(
                        f"Published: IMU(roll={roll}, pitch={pitch}, yaw={yaw}), "
                        f"Raw={raw_msg.data}"
                    )

            except json.JSONDecodeError:
                self.get_logger().warn(f"Non-JSON line: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

