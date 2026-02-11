import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32

topic1 = 'float_topc'
topic2 = 'string_topic'

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.sub_float = self.create_subscription(
            Float32,
            topic1,
            self.callback_fun1,
            10)

        self.sub_string = self.create_subscription(
            String,
            topic2,
            self.callback_fun2,
            10)
    
        # prevent unused variable warning
        self.sub_float
        self.sub_string

    def callback_fun1(self, msg):
        self.get_logger().info('Float message: "%s"' % msg.data)
    
    def callback_fun2(self, msg):
        self.get_logger().info('String message: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberNode()

    rclpy.spin(subscriber_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()