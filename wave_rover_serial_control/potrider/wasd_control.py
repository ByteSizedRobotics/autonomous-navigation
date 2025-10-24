import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select
import time

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

def get_key_nonblocking():
    """Check if a key is pressed without blocking."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def main(args=None):
    rclpy.init(args=args)
    node = WASDPublisher()
    toggle = 0
    speed = 2

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # non-blocking raw mode

    current_key = None
    last_command = None

    try:
        while rclpy.ok():
            key = get_key_nonblocking()

            if key is not None:
                current_key = key

            if current_key == 'w':
                last_command = f'{{"T":1,"L":{0.1 * speed},"R":{0.1 * speed}}}'
            elif current_key == 'a':
                last_command = f'{{"T":1,"L":{-0.25 * speed},"R":{0.25 * speed}}}'
            elif current_key == 's':
                last_command = f'{{"T":1,"L":{-0.1 * speed},"R":{-0.1 * speed}}}'
            elif current_key == 'd':
                last_command = f'{{"T":1,"L":{0.25 * speed},"R":{-0.25 * speed}}}'
            elif current_key == 'z':
                speed = min(speed + 0.5, 2)
                current_key = None
            elif current_key == 'x':
                speed = max(speed - 0.5, 1)
                current_key = None
            elif current_key == 'e':
                if toggle == 0:
                    toggle = 1
                    node.publish_command('{"T":126}')
                    node.publish_command('{"T":130}')
                else:
                    toggle = 0
                current_key = None
            elif current_key == 'q':
                break
            elif current_key is None:
                last_command = None

            # Continuous publish while key is held down
            if last_command:
                node.publish_command(last_command)

            # Handle toggle commands
            if toggle == 1:
                node.publish_command('{"T":126}')
                node.publish_command('{"T":130}')

            time.sleep(0.05)  # 20 Hz loop

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

