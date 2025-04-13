import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32, String
import numpy as np
import logging
from cv_bridge import CvBridge
import datetime


class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__("mock_camera_publisher")
        self.bridge = CvBridge()

        logging.basicConfig(level=logging.INFO)

        # Publishers for RGB, Depth, IR
        self.rgb_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(
            Image, "/camera/depth/image_rect_raw", 10
        )
        self.ir_pub = self.create_publisher(Image, "/camera/infra1/image_rect_raw", 10)

        # Environmental sensor data
        self.temperature_pub = self.create_publisher(Float32, "/camera/temperature", 10)
        self.exposure_pub = self.create_publisher(Float32, "/camera/exposure", 10)

        # Error simulation
        self.error_pub = self.create_publisher(String, "/camera/errors", 10)

        # Timers for stream publishing
        self.rgb_timer = self.create_timer(1.0 / 30.0, self.publish_rgb_image)  # 30 FPS
        self.depth_timer = self.create_timer(
            1.0 / 30.0, self.publish_depth_image
        )  # 30 FPS
        self.ir_timer = self.create_timer(1.0 / 90.0, self.publish_ir_image)  # 90 FPS

        # Timers for sensor data
        self.sensor_timer = self.create_timer(2.0, self.publish_environment_data)

    def generate_dummy_image(self, width, height, channels):
        return np.random.randint(0, 256, (height, width, channels), dtype=np.uint8)

    def publish_rgb_image(self):
        img = self.generate_dummy_image(1280, 720, 3)
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        ros_img.header = Header()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_rgb_frame"
        self.rgb_pub.publish(ros_img)

    def publish_depth_image(self):
        depth = np.random.randint(0, 10000, (720, 1280), dtype=np.uint16)
        ros_img = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
        ros_img.header = Header()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_depth_frame"
        self.depth_pub.publish(ros_img)

    def publish_ir_image(self):
        ir = self.generate_dummy_image(1280, 720, 1)
        ros_img = self.bridge.cv2_to_imgmsg(ir, encoding="mono8")
        ros_img.header = Header()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_ir_frame"
        self.ir_pub.publish(ros_img)

    def publish_environment_data(self):
        logger = logging.getLogger(__name__)

        # Simulate temperature (in °C) and exposure time (ms)
        temp_msg = Float32()
        temp_msg.data = np.random.uniform(40.0, 60.0)
        self.temperature_pub.publish(temp_msg)

        exposure_msg = Float32()
        exposure_msg.data = np.random.uniform(5.0, 20.0)
        self.exposure_pub.publish(exposure_msg)

        # Occasionally publish a fake error
        if np.random.rand() < 0.1:
            error_msg = String()
            error_msg.data = "Simulated stream error detected!"
            self.error_pub.publish(error_msg)
            system_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            logger.warning(f"{system_time} Published error: {error_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
