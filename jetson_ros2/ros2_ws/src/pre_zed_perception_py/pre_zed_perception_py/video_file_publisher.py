#!/usr/bin/env python3
"""Publish frames from one local MP4 as timestamped ROS 2 images."""

from pathlib import Path
import time

import cv2
import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from pre_zed_perception_py.contracts import stamp_to_nanoseconds
from pre_zed_perception_py.image_conversion import bgr_to_image_message


class VideoFilePublisher(Node):
    """Read a local video at a controlled rate and publish bgr8 images."""

    def __init__(self):
        super().__init__('video_file_publisher')
        self.declare_parameter('video_path', '')
        self.declare_parameter('output_topic', '/prezed/image_raw')
        self.declare_parameter('playback_rate_hz', 0.5)
        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'prezed_camera')
        self.declare_parameter('sequential_mode', False)
        self.declare_parameter('completion_topic', '/prezed/overlay')
        self.declare_parameter('frame_stride', 1)
        self.declare_parameter('completion_timeout_seconds', 150.0)
        self.declare_parameter('minimum_image_subscribers', 3)
        self.declare_parameter('startup_timeout_seconds', 30.0)
        self.declare_parameter('subscriber_settle_seconds', 2.0)
        self.declare_parameter('frame_retry_interval_seconds', 10.0)
        self.declare_parameter('max_frame_retries', 5)

        video_path = Path(
            str(self.get_parameter('video_path').value)
        ).expanduser()
        if not video_path.is_absolute():
            video_path = Path.cwd() / video_path
        if not video_path.is_file():
            raise RuntimeError(f'Video file does not exist: {video_path}')

        rate_hz = float(self.get_parameter('playback_rate_hz').value)
        if rate_hz <= 0.0:
            raise RuntimeError('playback_rate_hz must be greater than zero.')

        self.loop = bool(self.get_parameter('loop').value)
        self.sequential_mode = bool(
            self.get_parameter('sequential_mode').value
        )
        if self.sequential_mode and self.loop:
            raise RuntimeError(
                'sequential_mode requires loop=false so the run can finish.'
            )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.frame_stride = int(self.get_parameter('frame_stride').value)
        if self.frame_stride < 1:
            raise RuntimeError('frame_stride must be at least 1.')
        self.completion_timeout_seconds = float(
            self.get_parameter('completion_timeout_seconds').value
        )
        if self.completion_timeout_seconds <= 0.0:
            raise RuntimeError(
                'completion_timeout_seconds must be greater than zero.'
            )
        self.minimum_image_subscribers = max(
            1, int(self.get_parameter('minimum_image_subscribers').value)
        )
        self.startup_timeout_seconds = float(
            self.get_parameter('startup_timeout_seconds').value
        )
        if self.startup_timeout_seconds <= 0.0:
            raise RuntimeError(
                'startup_timeout_seconds must be greater than zero.'
            )
        self.subscriber_settle_seconds = float(
            self.get_parameter('subscriber_settle_seconds').value
        )
        if self.subscriber_settle_seconds < 0.0:
            raise RuntimeError(
                'subscriber_settle_seconds cannot be negative.'
            )
        self.frame_retry_interval_seconds = float(
            self.get_parameter('frame_retry_interval_seconds').value
        )
        if self.frame_retry_interval_seconds <= 0.0:
            raise RuntimeError(
                'frame_retry_interval_seconds must be greater than zero.'
            )
        self.max_frame_retries = max(
            0, int(self.get_parameter('max_frame_retries').value)
        )

        self.capture = cv2.VideoCapture(str(video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f'OpenCV could not open video: {video_path}')
        self.video_path = video_path

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            Image, output_topic, reliable_qos
        )
        self.timer = None
        self.startup_timer = None
        self.timeout_timer = None
        self.completion_subscription = None
        self.frame_count = 0
        self.completed_frame_count = 0
        self.frames_to_skip = 0
        self.pending_stamp_ns = None
        self.pending_message = None
        self.pending_started_at = None
        self.pending_last_published_at = None
        self.pending_retry_count = 0
        self.subscribers_ready_at = None
        self.finished = False

        if self.sequential_mode:
            completion_topic = str(
                self.get_parameter('completion_topic').value
            )
            self.completion_subscription = self.create_subscription(
                Image,
                completion_topic,
                self._on_frame_completed,
                reliable_qos,
            )
            self.startup_started_at = time.monotonic()
            self.startup_timer = self.create_timer(
                0.2, self._wait_for_subscribers
            )
            self.timeout_timer = self.create_timer(
                1.0, self._check_completion_timeout
            )
            self.get_logger().info(
                f'Finite sequential mode: {video_path} -> {output_topic}; '
                f'waiting for {self.minimum_image_subscribers} image '
                f'subscribers before publishing.'
            )
        else:
            self.timer = self.create_timer(
                1.0 / rate_hz, self._publish_frame
            )
            self.get_logger().info(
                f'Publishing {video_path} on {output_topic} at '
                f'{rate_hz:.3f} Hz.'
            )

    def _publish_frame(self):
        ok, frame = self._read_next_frame()
        if not ok:
            self.get_logger().info('Video ended; stopping frame publication.')
            self.timer.cancel()
            return

        self.publisher.publish(self._make_image_message(frame))
        self.frame_count += 1

    def _wait_for_subscribers(self):
        subscriber_count = self.publisher.get_subscription_count()
        if subscriber_count >= self.minimum_image_subscribers:
            if self.subscribers_ready_at is None:
                self.subscribers_ready_at = time.monotonic()
                self.get_logger().info(
                    f'Discovered {subscriber_count} image subscribers; '
                    f'waiting {self.subscriber_settle_seconds:.1f} seconds '
                    'for startup delivery readiness.'
                )
            settled_for = time.monotonic() - self.subscribers_ready_at
            if settled_for >= self.subscriber_settle_seconds:
                self.startup_timer.cancel()
                self.get_logger().info(
                    'Subscriber settling completed; starting finite '
                    'processing.'
                )
                self._publish_next_sequential_frame()
                return
        else:
            self.subscribers_ready_at = None

        elapsed = time.monotonic() - self.startup_started_at
        if elapsed >= self.startup_timeout_seconds:
            raise RuntimeError(
                f'Only discovered {subscriber_count} image subscribers after '
                f'{elapsed:.1f} seconds; expected at least '
                f'{self.minimum_image_subscribers}.'
            )

    def _publish_next_sequential_frame(self):
        ok, frame = self._read_next_frame()
        if not ok:
            self._finish_sequential_run()
            return

        message = self._make_image_message(frame)
        self.pending_stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        self.pending_message = message
        self.pending_started_at = time.monotonic()
        self.pending_last_published_at = self.pending_started_at
        self.pending_retry_count = 0
        self.publisher.publish(message)
        self.frame_count += 1
        self.get_logger().info(
            f'Published finite frame {self.frame_count} with stamp '
            f'{self.pending_stamp_ns}.'
        )

    def _on_frame_completed(self, message):
        if self.pending_stamp_ns is None:
            return
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if stamp_ns != self.pending_stamp_ns:
            return

        self.completed_frame_count += 1
        self.pending_stamp_ns = None
        self.pending_message = None
        self.pending_started_at = None
        self.pending_last_published_at = None
        self.pending_retry_count = 0
        self._publish_next_sequential_frame()

    def _check_completion_timeout(self):
        if self.pending_started_at is None:
            return
        elapsed = time.monotonic() - self.pending_started_at
        if elapsed >= self.completion_timeout_seconds:
            raise RuntimeError(
                f'Frame {self.frame_count} with stamp '
                f'{self.pending_stamp_ns} did not reach the overlay within '
                f'{elapsed:.1f} seconds.'
            )

        since_last_publish = (
            time.monotonic() - self.pending_last_published_at
        )
        if (
            since_last_publish >= self.frame_retry_interval_seconds
            and self.pending_retry_count < self.max_frame_retries
        ):
            self.pending_retry_count += 1
            self.pending_last_published_at = time.monotonic()
            self.publisher.publish(self.pending_message)
            self.get_logger().warning(
                f'No overlay acknowledgement for frame {self.frame_count}; '
                f'republished stamp {self.pending_stamp_ns} '
                f'(retry {self.pending_retry_count}/'
                f'{self.max_frame_retries}).'
            )

    def _finish_sequential_run(self):
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
        self.get_logger().info(
            f'Completed {self.completed_frame_count} frames from '
            f'{self.video_path}; shutting down the finite run.'
        )
        self.finished = True

    def _read_next_frame(self):
        while self.frames_to_skip > 0:
            if not self.capture.grab():
                return False, None
            self.frames_to_skip -= 1

        ok, frame = self.capture.read()
        if not ok and self.loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
        if ok:
            self.frames_to_skip = self.frame_stride - 1
        return ok, frame

    def _make_image_message(self, frame):
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return bgr_to_image_message(frame, header)

    def destroy_node(self):
        self.capture.release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    executor = None
    try:
        node = VideoFilePublisher()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
