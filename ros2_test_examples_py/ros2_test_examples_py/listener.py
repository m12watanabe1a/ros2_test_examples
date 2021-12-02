import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.callback, 10
        )

    def callback(self, msg: String):
        self.get_logger().info('I heard: [%s]' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node =Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()