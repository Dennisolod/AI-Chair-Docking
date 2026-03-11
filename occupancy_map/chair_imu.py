import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import depthai as dai
import sys

class OakdImuPublisher(Node):
    def __init__(self):
        super().__init__('oakd_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        print(">>> INITIALIZING PIPELINE...", flush=True)
        self.pipeline = dai.Pipeline()
        
        imu = self.pipeline.create(dai.node.IMU)
        xlink = self.pipeline.create(dai.node.XLinkOut)
        xlink.setStreamName("imu")
        
        # 50Hz is plenty for the wheelchair
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 50)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 50)
        imu.out.link(xlink.input)

        print(">>> OPENING DEVICE (USB HIGH SPEED)...", flush=True)
        try:
            # We use the speed confirmed by your test script
            self.device = dai.Device(self.pipeline, dai.UsbSpeed.HIGH)
            self.imu_queue = self.device.getOutputQueue(name="imu", maxSize=10, blocking=False)
            print(">>> SUCCESS: OAK-D CONNECTED!", flush=True)
        except Exception as e:
            print(f">>> ERROR: {e}", flush=True)
            sys.exit(1)
        
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        # Using tryGet prevents the node from freezing if no data is ready
        data = self.imu_queue.tryGet()
        
        if data is not None:
            for packet in data.packets:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                
                msg.linear_acceleration.x = packet.acceleroMeter.x
                msg.linear_acceleration.y = packet.acceleroMeter.y
                msg.linear_acceleration.z = packet.acceleroMeter.z
                
                msg.angular_velocity.x = packet.gyroscope.x
                msg.angular_velocity.y = packet.gyroscope.y
                msg.angular_velocity.z = packet.gyroscope.z
                
                msg.orientation.w = 1.0
                self.publisher_.publish(msg)
                
                # THIS IS YOUR VISUAL PROOF:
                print(f"Streaming IMU - Z-Accel: {packet.acceleroMeter.z:.2f}", end='\r', flush=True)
        else:
            # If the terminal stays here, the IMU chip is asleep
            print("Connected, but waiting for sensor data...", end='\r', flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = OakdImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()