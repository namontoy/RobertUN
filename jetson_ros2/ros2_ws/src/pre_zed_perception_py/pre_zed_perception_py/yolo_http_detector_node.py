#!/usr/bin/env python3
"""Run a configured YOLO detector through Roboflow HTTP inference."""

from collections import OrderedDict
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pre_zed_perception_py.contracts import (
    compact_json,
    detection_message,
    stamp_to_nanoseconds,
)
from pre_zed_perception_py.http_clients import InferenceError, YoloHttpClient
from pre_zed_perception_py.image_conversion import image_message_to_bgr


class YoloHttpDetectorNode(Node):
    """Publish object detections as a documented temporary JSON contract."""

    def __init__(self):
        super().__init__('yolo_http_detector_node')
        self.declare_parameter('input_topic', '/prezed/image_raw')
        self.declare_parameter('output_topic', '/prezed/detections')
        self.declare_parameter('server_url', 'http://localhost:9001')
        self.declare_parameter('detector_model_id', '')
        self.declare_parameter('api_key', '')
        self.declare_parameter('timeout_seconds', 120.0)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('allowed_classes', [''])

        model_id = str(self.get_parameter('detector_model_id').value).strip()
        if not model_id:
            raise RuntimeError(
                'detector_model_id is required. Set the Roboflow model ID in '
                'final_preZED/configs/pre_zed_params.yaml.'
            )
        api_key = str(self.get_parameter('api_key').value)
        if not api_key:
            api_key = os.environ.get('ROBOFLOW_API_KEY', '')

        self.model_id = model_id
        self.allowed_classes = [
            str(value)
            for value in self.get_parameter('allowed_classes').value
            if str(value)
        ]
        self.client = YoloHttpClient(
            server_url=str(self.get_parameter('server_url').value),
            model_id=model_id,
            api_key=api_key,
            timeout_seconds=float(self.get_parameter('timeout_seconds').value),
            jpeg_quality=int(self.get_parameter('jpeg_quality').value),
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').value
            ),
            iou_threshold=float(self.get_parameter('iou_threshold').value),
        )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter('input_topic').value),
            self._on_image,
            image_qos,
        )
        self.completed_stamps = OrderedDict()
        self.completed_stamp_cache_size = 64
        self.last_error_log_time = 0.0
        self.get_logger().info(
            f'Using detector model {self.model_id}; publishing {output_topic}.'
        )

    def _on_image(self, message):
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if stamp_ns in self.completed_stamps:
            return
        started = time.perf_counter()
        try:
            image = image_message_to_bgr(message)
            detections = self.client.infer(
                image,
                request_id=stamp_ns,
                allowed_classes=self.allowed_classes,
            )
        except (InferenceError, ValueError) as exc:
            self._log_error_throttled(str(exc))
            return

        inference_ms = (time.perf_counter() - started) * 1000.0
        payload = detection_message(
            stamp_ns=stamp_ns,
            frame_id=message.header.frame_id,
            width=message.width,
            height=message.height,
            model_id=self.model_id,
            detections=detections,
            inference_ms=inference_ms,
        )
        output = String()
        output.data = compact_json(payload)
        self.publisher.publish(output)
        self._mark_stamp_completed(stamp_ns)
        self.get_logger().info(
            f'Detected {len(detections)} objects in {inference_ms:.1f} ms.'
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
        node = YoloHttpDetectorNode()
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
