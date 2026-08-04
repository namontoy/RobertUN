#!/usr/bin/env python3
"""Render fused tracked objects on their matching source image."""

from collections import OrderedDict
import json
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pre_zed_perception_py.contracts import stamp_to_nanoseconds
from pre_zed_perception_py.image_conversion import (
    bgr_to_image_message,
    image_message_to_bgr,
    image_message_to_mono8,
)


class OverlayNode(Node):
    """Publish and optionally record the tracked-object visualization."""

    def __init__(self):
        super().__init__('pre_zed_overlay_node')
        self.declare_parameter('image_topic', '/prezed/image_raw')
        self.declare_parameter(
            'depth_topic', '/prezed/depth/relative_image'
        )
        self.declare_parameter('tracked_objects_topic', '/tracked_objects')
        self.declare_parameter('output_topic', '/prezed/overlay')
        self.declare_parameter('output_video_path', '')
        self.declare_parameter('output_video_fps', 1.0)
        self.declare_parameter('image_cache_size', 30)
        self.declare_parameter('draw_unobserved_tracks', False)
        self.declare_parameter('show_depth_inset', True)
        self.declare_parameter('depth_inset_width_ratio', 0.28)

        self.image_cache = OrderedDict()
        self.track_cache = OrderedDict()
        self.depth_cache = OrderedDict()
        self.completed_stamps = OrderedDict()
        self.image_cache_size = max(
            2, int(self.get_parameter('image_cache_size').value)
        )
        self.completed_stamp_cache_size = max(
            64, self.image_cache_size * 2
        )
        self.draw_unobserved_tracks = bool(
            self.get_parameter('draw_unobserved_tracks').value
        )
        self.show_depth_inset = bool(
            self.get_parameter('show_depth_inset').value
        )
        self.depth_inset_width_ratio = min(
            0.5,
            max(
                0.15,
                float(
                    self.get_parameter(
                        'depth_inset_width_ratio'
                    ).value
                ),
            ),
        )
        self.output_video_path = str(
            self.get_parameter('output_video_path').value
        ).strip()
        self.output_video_fps = max(
            0.1, float(self.get_parameter('output_video_fps').value)
        )
        self.writer = None
        self.resolved_output_video_path = None
        self.written_frame_count = 0
        self.last_warning_time = 0.0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter('output_topic').value),
            reliable_qos,
        )
        self.image_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            image_qos,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            reliable_qos,
        )
        self.tracks_subscription = self.create_subscription(
            String,
            str(self.get_parameter('tracked_objects_topic').value),
            self._on_tracks,
            10,
        )

    def _on_image(self, message):
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if stamp_ns in self.completed_stamps:
            return
        try:
            image = image_message_to_bgr(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        self.image_cache[stamp_ns] = (message.header, image)
        self.image_cache.move_to_end(stamp_ns)
        self._trim_cache(self.image_cache)
        self._try_render(stamp_ns)

    def _on_tracks(self, message):
        try:
            payload = json.loads(message.data)
            stamp_ns = int(payload['stamp_ns'])
            tracks = payload['tracks']
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid /tracked_objects JSON: {exc}')
            return

        if stamp_ns in self.completed_stamps:
            return
        self.track_cache[stamp_ns] = tracks
        self.track_cache.move_to_end(stamp_ns)
        self._trim_cache(self.track_cache)
        self._try_render(stamp_ns)

    def _on_depth(self, message):
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if stamp_ns in self.completed_stamps:
            return
        try:
            depth = image_message_to_mono8(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        self.depth_cache[stamp_ns] = depth
        self.depth_cache.move_to_end(stamp_ns)
        self._trim_cache(self.depth_cache)
        self._try_render(stamp_ns)

    def _try_render(self, stamp_ns):
        cached_image = self.image_cache.get(stamp_ns)
        tracks = self.track_cache.get(stamp_ns)
        depth = self.depth_cache.get(stamp_ns)
        if cached_image is None or tracks is None or depth is None:
            if tracks is not None:
                missing = []
                if cached_image is None:
                    missing.append('source image')
                if depth is None:
                    missing.append('relative depth')
                self._warn_incomplete(stamp_ns, missing)
            return

        header, image = self.image_cache.pop(stamp_ns)
        tracks = self.track_cache.pop(stamp_ns)
        depth = self.depth_cache.pop(stamp_ns)
        self._render_overlay(header, image, tracks, depth)
        self._mark_completed(stamp_ns)

    def _render_overlay(self, header, image, tracks, depth):
        overlay = image.copy()
        visible_tracks = [
            track
            for track in tracks
            if self.draw_unobserved_tracks
            or bool(track.get('observed', True))
        ]
        for track in visible_tracks:
            self._draw_track(overlay, track)
        if self.show_depth_inset:
            self._draw_depth_inset(overlay, depth)

        self._write_video_frame(overlay)
        self.publisher.publish(bgr_to_image_message(overlay, header))

    def _trim_cache(self, cache):
        while len(cache) > self.image_cache_size:
            cache.popitem(last=False)

    def _mark_completed(self, stamp_ns):
        self.completed_stamps[stamp_ns] = None
        self.completed_stamps.move_to_end(stamp_ns)
        while len(self.completed_stamps) > self.completed_stamp_cache_size:
            self.completed_stamps.popitem(last=False)

    def _draw_track(self, image, track):
        try:
            track_id = int(track['track_id'])
            label = str(track['label'])
            confidence = float(track['confidence'])
            left, top, width, height = [
                float(value) for value in track['bbox_xywh']
            ]
        except (KeyError, TypeError, ValueError):
            return

        x1 = max(0, int(round(left)))
        y1 = max(0, int(round(top)))
        x2 = min(image.shape[1] - 1, int(round(left + width)))
        y2 = min(image.shape[0] - 1, int(round(top + height)))
        color = self._track_color(track_id)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        captions = [f'#{track_id} {label} {confidence:.2f}']
        if bool(track.get('relative_depth_valid', False)):
            depth = float(track['relative_depth_median'])
            captions.append(f'Relative depth: {depth:.3f}')
        else:
            captions.append('Relative depth: unavailable')
        self._draw_caption_block(image, x1, y1, captions, color)

    @staticmethod
    def _draw_caption_block(image, x, box_top, lines, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        thickness = 1
        line_height = 18
        padding = 5
        text_width = max(
            cv2.getTextSize(
                line, font, font_scale, thickness
            )[0][0]
            for line in lines
        )
        block_width = text_width + padding * 2
        block_height = len(lines) * line_height + padding * 2
        left = min(
            max(0, x),
            max(0, image.shape[1] - block_width - 1),
        )
        if box_top >= block_height + 2:
            top = box_top - block_height
        else:
            top = max(0, box_top)
        right = min(image.shape[1] - 1, left + block_width)
        bottom = min(image.shape[0] - 1, top + block_height)
        cv2.rectangle(image, (left, top), (right, bottom), color, -1)
        for index, line in enumerate(lines):
            baseline = top + padding + 13 + index * line_height
            cv2.putText(
                image,
                line,
                (left + padding, baseline),
                font,
                font_scale,
                (15, 15, 15),
                thickness,
                cv2.LINE_AA,
            )

    def _draw_depth_inset(self, image, depth):
        margin = 12
        label_height = 42
        image_height, image_width = image.shape[:2]
        inset_width = int(round(
            image_width * self.depth_inset_width_ratio
        ))
        inset_width = min(
            max(120, inset_width),
            max(120, image_width - margin * 2),
        )
        scale = inset_width / float(depth.shape[1])
        inset_height = max(1, int(round(depth.shape[0] * scale)))
        maximum_height = max(1, image_height - margin * 2 - label_height)
        if inset_height > maximum_height:
            inset_height = maximum_height
            inset_width = max(
                1,
                int(round(depth.shape[1] * inset_height / depth.shape[0])),
            )

        color_depth = cv2.applyColorMap(depth, cv2.COLORMAP_INFERNO)
        color_depth = cv2.resize(
            color_depth,
            (inset_width, inset_height),
            interpolation=cv2.INTER_AREA,
        )
        left = image_width - inset_width - margin
        label_top = margin
        image_top = label_top + label_height
        image[
            image_top:image_top + inset_height,
            left:left + inset_width,
        ] = color_depth
        cv2.rectangle(
            image,
            (left - 1, label_top - 1),
            (left + inset_width, image_top + inset_height),
            (245, 245, 245),
            2,
        )
        cv2.rectangle(
            image,
            (left, label_top),
            (left + inset_width - 1, image_top - 1),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            image,
            'RELATIVE DEPTH',
            (left + 6, label_top + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            'NORMALIZED / NOT METERS',
            (left + 6, label_top + 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _track_color(track_id):
        return (
            int(40 + (track_id * 67) % 190),
            int(40 + (track_id * 101) % 190),
            int(40 + (track_id * 149) % 190),
        )

    def _write_video_frame(self, image):
        if not self.output_video_path:
            return
        if self.writer is None:
            path = Path(self.output_video_path).expanduser()
            if not path.is_absolute():
                path = Path.cwd() / path
            path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(
                str(path),
                fourcc,
                self.output_video_fps,
                (image.shape[1], image.shape[0]),
            )
            if not self.writer.isOpened():
                self.writer = None
                self.get_logger().error(
                    f'Could not create overlay video: {path}'
                )
                self.output_video_path = ''
                return
            self.resolved_output_video_path = path
            self.get_logger().info(f'Recording overlay video to {path}.')
        self.writer.write(image)
        self.written_frame_count += 1

    def _warn_incomplete(self, stamp_ns, missing):
        if not missing:
            return
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(
                f'Waiting for {", ".join(missing)} for tracked frame '
                f'{stamp_ns}; a source retry may complete synchronization.'
            )
            self.last_warning_time = now

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            print(
                f'Finalized {self.resolved_output_video_path} with '
                f'{self.written_frame_count} frames.',
                flush=True,
            )
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = OverlayNode()
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
