import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Float32


class BatteryCheck(Node):
    def __init__(self):
        super().__init__('battery_check')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial port
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # Publisher
        self.publisher_ = self.create_publisher(Float32, 'battery_percent', 10)

        # Timer to poll every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Enable continuous feedback mode on startup
        self.enable_feedback(True)

    def enable_feedback(self, enable: bool):
        # Enable or disable continuous feedback from rover
        
        if enable:
            cmd = {"T":131,"cmd":1}
        else:
            cmd = {"T":131,"cmd":0}
        try:
            self.ser.write((json.dumps(cmd) + '\n').encode('utf-8'))
            self.get_logger().info(f"{'Enabled' if enable else 'Disabled'} continuous feedback")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def timer_callback(self):
        try:
            # Send poll json cmd
            cmd = {"T": 130}
            self.ser.write((json.dumps(cmd) + '\n').encode('utf-8'))
            
            # Read response
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            data = json.loads(line)
            """
            if "v" in data:
                raw_v = data["v"]

                # Convert voltage → percentage
                level = (raw_v - 9.0) / (12.6 - 9.0) * 100.0
                level = max(0.0, min(100.0, level))
            """
            
            msg = Float32()
            msg.data = data
            self.publisher_.publish(msg)

            # self.get_logger().info(f"Battery: {raw_v:.2f} V → {level:.1f}%")

        except json.JSONDecodeError:
            pass  # ignore junk lines
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def destroy_node(self):
        """Clean shutdown: disable feedback and close serial."""
        self.enable_feedback(False)
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

