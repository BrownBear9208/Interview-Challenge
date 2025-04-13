import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import pytest
import time
import logging

received_messages = []
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class StreamSubscriber(Node):
    def __init__(self):
        super().__init__('camera_mock_collector')
        

        # Data containers
        self.rgb_data = []
        self.depth_data = []
        self.ir_data = []
        self.temp_data = []
        self.exposure_data = []
        self.errors = []
        self.error_timestamps = []

        # Subscriptions
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/infra1/image_rect_raw', self.ir_callback, 10)
        self.create_subscription(Float32, '/camera/temperature', self.temp_callback, 10)
        self.create_subscription(Float32, '/camera/exposure', self.exposure_callback, 10)
        self.create_subscription(String, '/camera/errors', self.error_callback, 10)

    def rgb_callback(self, msg):
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self.rgb_data.append(timestamp)

    def depth_callback(self, msg):
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self.depth_data.append(timestamp)

    def ir_callback(self, msg):
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self.ir_data.append(timestamp)

    def temp_callback(self, msg):
        self.temp_data.append(msg.data)

    def exposure_callback(self, msg):
        self.exposure_data.append(msg.data)

    def error_callback(self, msg):
        error_msg = msg.data

        # Store the message
        self.errors.append(error_msg)

        # Store the time it was received
        now = self.get_clock().now()
        timestamp = now.nanoseconds * 1e-9  # nanoseconds → seconds
        self.error_timestamps.append(timestamp)


def run_ros_node(duration=5):
    rclpy.init()
    node = StreamSubscriber()
    try:
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rclpy.shutdown()
    return node

@pytest.mark.timeout(10)
def test_all_streams_are_publishing():
    """
    Test that all critical camera streams and sensor data topics are actively publishing.

    This test ensures that each of the six expected data streams—
    RGB image, depth image, infrared image, temperature, and exposure—are emitting messages
    within a 5-second window after node initialization.

    A failure in any of these assertions likely indicates an issue with the publisher node or topic configuration.
    """
    node = run_ros_node(5)

    assert len(node.rgb_data) > 0, "RGB stream not publishing"
    assert len(node.depth_data) > 0, "Depth stream not publishing"
    assert len(node.ir_data) > 0, "IR stream not publishing"
    assert len(node.temp_data) > 0, "Temperature data not publishing"
    assert len(node.exposure_data) > 0, "Exposure data not publishing"

@pytest.mark.timeout(10)
def test_temperature_values_in_range():
    """
    Test that simulated temperature readings fall within expected operational thresholds.

    The expected temperature range is 30°C to 70°C. This test ensures that the simulation
    does not produce extreme or invalid sensor values, which might affect downstream logic.
    """
    node = run_ros_node(5)

    assert all(30.0 <= temp <= 70.0 for temp in node.temp_data), \
        "Temperature values are out of expected range (30–70°C)"


@pytest.mark.timeout(10)
def test_exposure_values_in_range():
    """
    Test that simulated exposure values fall within acceptable operational limits.

    The exposure time, in milliseconds, is expected to remain between 1 and 30 ms.
    Out-of-range values may indicate simulation errors or misconfigured publishers.
    """
    node = run_ros_node(5)

    assert all(1.0 <= exp <= 30.0 for exp in node.exposure_data), \
        "Exposure values are out of expected range (1–30ms)"


@pytest.mark.timeout(10)
def test_simulated_errors_might_appear():
    """
    Test that the error simulation occasionally triggers and publishes an error message.

    The mock publisher simulates errors with a 10% probability. This test observes the system
    for 10 seconds and verifies whether any error messages are received. Failure to receive errors
    is not necessarily a fault, but a warning is logged if none are observed.
    """
    node = run_ros_node(10)

    if not node.errors:
        logger.warning("No simulated error received, but that's probably fine.")
    else:
        assert "error" in node.errors[0].lower(), "First error message doesn't look like an error"


@pytest.mark.timeout(10)
def test_rgb_stream_frequency_approximate():
    """
    Test that the RGB image stream publishes at an approximate rate of 30 frames per second.

    Calculates the average time interval between consecutive RGB frames.
    Acceptable range is 20ms to 50ms (corresponding to 20–50 FPS), with a target of ~33ms.
    Large deviations could indicate performance issues or incorrect timer settings.
    """
    node = run_ros_node(5)

    intervals = [
        t2 - t1 for t1, t2 in zip(node.rgb_data, node.rgb_data[1:])
    ]
    average_interval = sum(intervals) / len(intervals) if intervals else 0

    assert 0.02 < average_interval < 0.05, \
        f"RGB stream interval seems off: avg = {average_interval:.3f}s"

