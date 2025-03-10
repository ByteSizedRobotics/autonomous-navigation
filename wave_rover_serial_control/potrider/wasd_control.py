import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class WASDPublisher(Node):
    def __init__(self):
        super().__init__('wasd_control')
        self.publisher_ = self.create_publisher(String, '/JSON', 10)
        self.get_logger().info('Use WASD keys to control. Press q to quit.')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = WASDPublisher()

    try:
        while True:
            key = get_key()
            if key == 'w':
                command = '{"T":1,"L":0.1,"R":0.1}'
                node.publish_command(command)
            elif key == 'a':
                command = '{"T":1,"L":-0.25,"R":0.25}'
                node.publish_command(command)
            elif key == 's':
                command = '{"T":1,"L":-0.1,"R":-0.1}'
                node.publish_command(command)
            elif key == 'd':
                command = '{"T":1,"L":0.25,"R":-0.25}'
                node.publish_command(command)
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
