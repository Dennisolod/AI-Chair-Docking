import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32

topic1 = 'float_topc'
topic2 = 'string_topic'

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')

        self.pub_float = self.create_publisher(Float32, topic1, 10)
        self.pub_string = self.create_publisher(String, topic2, 10)

        self.period = 1  # seconds
        self.timer = self.create_timer(self.period, self.callback_fun)

        self.counter = 0
        self.value = 0

    def callback_fun(self):
        message1 = Float32()
        message2 = String()

        self.value = self.value + 0.01
        message1.data = self.value
        message2.data = 'Message number: %d' % self.counter
        
        self.pub_string.publish(message2)
        self.pub_float.publish(message1)

        self.get_logger().info('publishing: %s and value %s' 
        % (message2.data, message1.data))
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()