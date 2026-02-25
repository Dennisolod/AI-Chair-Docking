import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import depthai as dai
import math

class OakdImuPublisher(Node):
    def __init__(self):
        super().__init__('oakd_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Setup OAK-D Pipeline
        self.pipeline = dai.Pipeline()
        imu = self.pipeline.create(dai.node.IMU)
        
        # Standard sensors for OAK-D IMU
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        
        imu_out = self.pipeline.create(dai.node.XLinkOut)
        imu_out.setStreamName("imu")
        imu.out.link(imu_out.input)

        # Connect to device and start pipeline
        self.get_logger().info("Connecting to OAK-D camera...")
        try:
            self.device = dai.Device(self.pipeline)
            self.imu_queue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            self.get_logger().info("OAK-D Connected. Publishing IMU data.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to OAK-D: {e}")
            raise e
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.imu_queue.has():
            imu_data = self.imu_queue.get()
            for packet in imu_data.packets:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                
                # Accel (m/s^2)
                msg.linear_acceleration.x = packet.acceleroMeter.x
                msg.linear_acceleration.y = packet.acceleroMeter.y
                msg.linear_acceleration.z = packet.acceleroMeter.z
                
                # Gyro (rad/s)
                msg.angular_velocity.x = packet.gyroscope.x
                msg.angular_velocity.y = packet.gyroscope.y
                msg.angular_velocity.z = packet.gyroscope.z
                
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OakdImuPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node stopped: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()