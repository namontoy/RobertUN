#!/usr/bin/env python3
"""Run Depth Anything V3 over HTTP for timestamped ROS 2 images."""

from collections import OrderedDict
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from pre_zed_perception_py.contracts import stamp_to_nanoseconds
from pre_zed_perception_py.http_clients import (
    DepthAnythingHttpClient,
    InferenceError,
    normalize_depth_to_mono8,
)
from pre_zed_perception_py.image_conversion import (
    bgr_to_image_message,
    image_message_to_bgr,
    mono8_to_image_message,
)


class DepthAnythingHttpNode(Node):
    """Publish normalized relative depth and a color visualization."""

    def __init__(self):
        super().__init__('depth_anything_http_node')
        self.declare_parameter('input_topic', '/prezed/image_raw')
        self.declare_parameter(
            'relative_depth_topic', '/prezed/depth/relative_image'
        )
        self.declare_parameter(
            'visualization_topic', '/prezed/depth/visualization'
        )
        self.declare_parameter('server_url', 'http://localhost:9001')
        self.declare_parameter('model_id', 'depth-anything-v3/small')
        self.declare_parameter('api_key', '')
        self.declare_parameter('timeout_seconds', 120.0)
        self.declare_parameter('jpeg_quality', 90)

        api_key = str(self.get_parameter('api_key').value)
        if not api_key:
            api_key = os.environ.get('ROBOFLOW_API_KEY', '')
        self.client = DepthAnythingHttpClient(
            server_url=str(self.get_parameter('server_url').value),
            model_id=str(self.get_parameter('model_id').value),
            api_key=api_key,
            timeout_seconds=float(self.get_parameter('timeout_seconds').value),
            jpeg_quality=int(self.get_parameter('jpeg_quality').value),
        )

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        visualization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.depth_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('relative_depth_topic').value),
            source_qos,
        )
        self.visualization_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('visualization_topic').value),
            visualization_qos,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter('input_topic').value),
            self._on_image,
            source_qos,
        )
        self.completed_stamps = OrderedDict()
        self.completed_stamp_cache_size = 64
        self.last_error_log_time = 0.0

    def _on_image(self, message):
        request_id = stamp_to_nanoseconds(message.header.stamp)
        if request_id in self.completed_stamps:
            return
        started = time.perf_counter()
        try:
            image = image_message_to_bgr(message)
            raw_depth = self.client.infer(image, request_id)
            mono8 = normalize_depth_to_mono8(
                raw_depth,
                output_width=message.width,
                output_height=message.height,
            )
        except (InferenceError, ValueError) as exc:
            self._log_error_throttled(str(exc))
            return

        depth_message = mono8_to_image_message(mono8, message.header)
        visualization = cv2.applyColorMap(mono8, cv2.COLORMAP_INFERNO)
        visualization_message = bgr_to_image_message(
            visualization, message.header
        )
        self.depth_publisher.publish(depth_message)
        self.visualization_publisher.publish(visualization_message)
        self._mark_stamp_completed(request_id)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f'Depth frame {request_id} completed in {elapsed_ms:.1f} ms.'
        )

    def _log_error_throttled(self, message):
        now = time.monotonic()
        if now - self.last_error_log_time >= 5.0:
            self.get_logger().error(message)
            self.last_error_log_time = now

    def _mark_stamp_completed(self, stamp_ns):
        self.completed_stamps[stamp_ns] = None
        self.completed_stamps.move_to_end(stamp_ns)
        while len(self.completed_stamps) > self.completed_stamp_cache_size:
            self.completed_stamps.popitem(last=False)


def main():
    rclpy.init()
    node = None
    try:
        node = DepthAnythingHttpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
