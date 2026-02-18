import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class DummyController(Node):
    def __init__(self):
        super().__init__('dummy_controller')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Internal state (x, y, theta)
        self.x = 0.0
        self.y = 3.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update_pose)

    def cmd_vel_callback(self, msg):
        # We use the velocity commands from Nav2
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_pose(self):
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        self.last_time = curr_time

        # Update position based on velocity
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Broadcast the 'odom' to 'base_link' transform
        t = TransformStamped()
        t.header.stamp = curr_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.br.sendTransform(t)

    vx = 0.0
    vth = 0.0

def main():
    rclpy.init()
    node = DummyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()