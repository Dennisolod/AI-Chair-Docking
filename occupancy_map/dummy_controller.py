import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration
import math

class DummyController(Node):
    def __init__(self):
        super().__init__('dummy_controller')
        self.br = TransformBroadcaster(self)
        
        # Publisher for AMCL/Nav2 to track movement
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscriber for movement commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_pose)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_pose(self):
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        self.last_time = curr_time

        # Calculate dead reckoning
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        # 1. Broadcast TF (odom -> base_link) for RViz visualization
        t = TransformStamped()
        t.header.stamp = curr_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.br.sendTransform(t)

        # 2. Publish Odometry message for AMCL localization
        odom = Odometry()
        odom.header.stamp = curr_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = DummyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
