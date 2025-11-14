import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class CmdVelToJSON(Node):
    def __init__(self):
        super().__init__('cmdvel_to_json')

        # Parameters
        self.declare_parameter('port', '/dev/rover_serial') # TODO: NATHAN verify this
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        # Subscribe to velocity commands
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.listener, 10)

    def listener(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial port not available")
            return

        # Convert cmd_vel → left/right motor speeds
        linear = msg.linear.x   # forward speed (m/s)
        angular = msg.angular.z # rotation (rad/s)

        base_speed = linear
        turn = angular * 0.5  # tune scaling factor
        left = base_speed - turn
        right = base_speed + turn

        # Clamp values to [-1, 1] (assuming rover expects normalized commands)
        cmd = {
            "T": 1,
            "L": max(-1.0, min(1.0, left)),
            "R": max(-1.0, min(1.0, right))
        }

        try:
            msg_str = json.dumps(cmd) + "\n"
            self.ser.write(msg_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Failed to send motor command: {e}")


def main():
    rclpy.init()
    node = CmdVelToJSON()
    rclpy.spin(node)
    if node.ser is not None and node.ser.is_open:
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

