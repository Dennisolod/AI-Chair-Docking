#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf2_ros
from tf2_ros import TransformException


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeLidarFromMap(Node):
    def __init__(self):
        super().__init__('fake_lidar_from_map')

        # Topics
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initialpose_topic', '/initialpose')

        # Frames
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_frame', 'laser_frame')

        # Scan settings
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('num_rays', 180)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)

        # Occupancy interpretation
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('treat_unknown_as_occupied', False)

        # Auto-initial pose (helps AMCL start without clicking RViz)
        self.declare_parameter('auto_initialpose', True)
        self.declare_parameter('initialpose_x', 0.0)
        self.declare_parameter('initialpose_y', 0.0)
        self.declare_parameter('initialpose_yaw', 0.0)

        # QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        init_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.map_cb,
            map_qos
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            scan_qos
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initialpose_topic').value),
            init_qos
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map: Optional[OccupancyGrid] = None
        self._warned_fallback = False
        self._initialpose_sent = False

        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_scan)

        self.get_logger().info(
            "Fake lidar (map raycast) started.\n"
            "- Prefers pose in 'map' frame.\n"
            "- Falls back to 'odom' pose until map TF exists.\n"
            "- Can auto-publish /initialpose once.\n"
        )

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        # Send initial pose once after receiving the map (optional)
        if (not self._initialpose_sent) and bool(self.get_parameter('auto_initialpose').value):
            self.publish_initial_pose()
            self._initialpose_sent = True

    def publish_initial_pose(self):
        map_frame = str(self.get_parameter('map_frame').value)
        x = float(self.get_parameter('initialpose_x').value)
        y = float(self.get_parameter('initialpose_y').value)
        yaw = float(self.get_parameter('initialpose_yaw').value)
        qx, qy, qz, qw = quat_from_yaw(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Reasonable "I'm not super sure" covariance
        cov = [0.0] * 36
        cov[0] = 0.5 * 0.5     # x
        cov[7] = 0.5 * 0.5     # y
        cov[35] = (math.radians(30.0) ** 2)  # yaw
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        self.get_logger().warn(
            f"Auto-published /initialpose at x={x}, y={y}, yaw={yaw} (rad) in frame '{map_frame}'. "
            "If this isn't near your true start, set 2D Pose Estimate in RViz once."
        )

    def world_to_grid(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        if self.map is None:
            return None
        info = self.map.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def occ_at(self, mx: int, my: int) -> int:
        assert self.map is not None
        idx = my * self.map.info.width + mx
        return int(self.map.data[idx])

    def raycast(self, x: float, y: float, theta: float, rmax: float) -> float:
        assert self.map is not None
        info = self.map.info
        step = info.resolution * 0.5
        occ_thr = int(self.get_parameter('occupied_threshold').value)
        unk_occ = bool(self.get_parameter('treat_unknown_as_occupied').value)
        rmin = float(self.get_parameter('range_min').value)

        r = rmin
        while r <= rmax:
            wx = x + r * math.cos(theta)
            wy = y + r * math.sin(theta)
            g = self.world_to_grid(wx, wy)
            if g is None:
                return r
            mx, my = g
            v = self.occ_at(mx, my)
            if v == -1 and unk_occ:
                return r
            if v >= occ_thr:
                return r
            r += step
        return rmax

    def _lookup_pose(self):
        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)

        # Ask for the latest available TF (time=0), give it a small window to arrive
        t = rclpy.time.Time()
        timeout = rclpy.duration.Duration(seconds=0.05)

        # Prefer pose in map frame if available, otherwise fall back to odom frame
        used_frame = None
        if self.tf_buffer.can_transform(map_frame, base_frame, t, timeout):
            used_frame = map_frame
        elif self.tf_buffer.can_transform(odom_frame, base_frame, t, timeout):
            used_frame = odom_frame

        if used_frame is None:
            raise TransformException("No transform available: (map or odom) -> base_link")

        tf = self.tf_buffer.lookup_transform(used_frame, base_frame, t)
        stamp = tf.header.stamp

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        return x, y, yaw, used_frame, stamp

    def publish_scan(self):
        if self.map is None:
            return

        scan_frame = str(self.get_parameter('scan_frame').value)
        n = int(self.get_parameter('num_rays').value)
        rmin = float(self.get_parameter('range_min').value)
        rmax = float(self.get_parameter('range_max').value)

        try:
            x, y, yaw, _, tf_stamp = self._lookup_pose()
        except TransformException:
            return

        msg = LaserScan()
        msg.header.stamp = tf_stamp
        msg.header.frame_id = scan_frame

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / float(n)

        rate = float(self.get_parameter('rate_hz').value)
        msg.scan_time = 1.0 / rate
        msg.time_increment = msg.scan_time / float(n)

        msg.range_min = rmin
        msg.range_max = rmax

        ranges = []
        a = msg.angle_min
        for _ in range(n):
            theta = yaw + a
            ranges.append(self.raycast(x, y, theta, rmax))
            a += msg.angle_increment

        msg.ranges = ranges
        msg.intensities = []

        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidarFromMap()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
