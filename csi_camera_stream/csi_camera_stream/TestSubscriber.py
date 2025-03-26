import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'csi_video_stream',
            self.callback,
            10
        )
        self.get_logger().info("Subscribed to csi_video_stream")

    def callback(self, msg):
        self.get_logger().info("Received a message")

def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()