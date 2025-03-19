import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_motor_node')
        
        # Declare and get parameters
        self.declare_parameter('port', '/dev/rover_serial')
        self.port = self.get_parameter('port').get_parameter_value().string_value

        # Initialize serial connection
        self.ser = serial.Serial(self.port, baudrate=115200, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)

        # Subscriber to JSON topic
        self.subscription = self.create_subscription(
            String,
            '/JSON',
            self.listener_callback,
            10
        )
        
        # Initializes the last message time and the default command
        self.last_message_time = time.time()
        self.default_command = '{"T":1,"L":0,"R":0}'
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Start the timing thread to send default command
        self.timer_thread = threading.Thread(target=self.send_default_command)
        self.timer_thread.daemon = True
        self.timer_thread.start()

    def listener_callback(self, msg):
        try:
            # Write received JSON string to serial port
            self.ser.write(msg.data.encode() + b'\n')
            self.get_logger().info(f'Sent: {msg.data}')  # Only log received commands
            self.last_message_time = time.time()  # Reset the timer
        except Exception as e:
            self.get_logger().error(f'Error sending data: {e}')

    def read_serial(self):
        while rclpy.ok():
            try:
                data = self.ser.readline().decode('utf-8').strip()
                if data:
                    self.get_logger().info(f'Received: {data}')
            except Exception as e:
                self.get_logger().error(f'Error reading data: {e}')

    def send_default_command(self):
        while rclpy.ok():
            time.sleep(1)  # Check every 100 ms
            if time.time() - self.last_message_time > 0.1:  # If no message for 100 ms 
                try:
                    self.ser.write(self.default_command.encode() + b'\n')
                    # Optionally log here if you want to see default message being sent
                    # self.get_logger().info(f'Sent default command: {self.default_command}')
                except Exception as e:
                    self.get_logger().error(f'Error sending default command: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
