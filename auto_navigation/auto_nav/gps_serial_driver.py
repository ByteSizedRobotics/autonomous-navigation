#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GPSSerialNode(Node):
    def __init__(self):
        super().__init__('gps_serial_node')

        # Create publisher on /fix
        self.publisher_ = self.create_publisher(String, '/fix', 10)

        # Open serial port
        try:
            self.ser = serial.Serial('/dev/GPS_serial', 9600, timeout=1)
            self.get_logger().info("Opened /dev/GPS_serial @ 9600 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open /dev/GPS_serial: {e}")
            raise SystemExit

        # Timer to check serial data
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().debug(f"Published: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

