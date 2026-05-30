#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from image_tools_py.color_car_detector import VALID_COLORS, count_cars_by_color


class ColorCarCounter(Node):
    """Count likely cars by simple color segmentation in one local image."""

    def __init__(self):
        super().__init__('color_car_counter')

        self.declare_parameter('image_path', '')
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_area', 450.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.image_path = str(self.get_parameter('image_path').value)
        self.target_color = str(self.get_parameter('target_color').value).lower()
        self.min_area = float(self.get_parameter('min_area').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if publish_rate_hz <= 0:
            publish_rate_hz = 1.0

        self.count_pub = self.create_publisher(Int32, '/color_car_count', 10)
        self.summary_pub = self.create_publisher(String, '/color_car_summary', 10)

        self.detected_count = 0
        self.summary = ''
        self._process_image()

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_result)

    def _process_image(self):
        result = count_cars_by_color(
            self.image_path,
            self.target_color,
            min_area=self.min_area,
        )

        if not result.success:
            valid = ', '.join(VALID_COLORS)
            self.get_logger().error(f"{result.summary}. Valid colors: {valid}")
            return

        self.detected_count = result.count
        self.summary = result.summary

        self.get_logger().info(result.summary)
        self.get_logger().info(
            'This is a color-segmentation estimate, not a trained car detector.'
        )

    def _publish_result(self):
        count_msg = Int32()
        count_msg.data = self.detected_count
        self.count_pub.publish(count_msg)

        summary_msg = String()
        summary_msg.data = self.summary
        self.summary_pub.publish(summary_msg)


def main():
    rclpy.init()
    node = ColorCarCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
