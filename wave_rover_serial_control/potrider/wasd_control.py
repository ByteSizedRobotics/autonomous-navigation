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
    toggle = 0
    speed = 1

    try:
        while True:
            key = get_key()
            if key == 'w':
                command = '{"T":1,"L":%s,"R":%s}' % (0.1 * speed, 0.1 * speed)
                node.publish_command(command)
            elif key == 'a':
                command = '{"T":1,"L":%s,"R":%s}' % (-0.25 * speed, 0.25 * speed)
                node.publish_command(command)
            elif key == 's':
                command = '{"T":1,"L":%s,"R":%s}' % (-0.1 * speed, -0.1 * speed)
                node.publish_command(command)
            elif key == 'd':
                command = '{"T":1,"L":%s,"R":%s}' % (0.25 * speed, -0.25 * speed)
                node.publish_command(command)
            elif key == 'z':
                if speed == 2:
                    pass
                else:
                    speed += 0.5
            elif key == 'x':
                if speed == 1:
                    pass
                else:
                    speed -=0.5            
            elif key == 'e':
                if toggle == 0:
                    toggle = 1
                    command = '{"T":126}'
                    node.publish_command(command)
                    command = '{"T":130}'
                    node.publish_command(command)
                elif toggle == 1:
                    toggle = 0
            elif key == 'q':
                break
            if toggle == 1:
                command = '{"T":126}'
                node.publish_command(command)
                command = '{"T":130}'
                node.publish_command(command)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
