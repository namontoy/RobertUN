#!/usr/bin/env python3

from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class ColorCarCounter(Node):
    """Count likely cars by simple color segmentation in one local image."""

    VALID_COLORS = ('red', 'white', 'black')

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
        if self.target_color not in self.VALID_COLORS:
            valid = ', '.join(self.VALID_COLORS)
            self.get_logger().error(
                f"Invalid target_color='{self.target_color}'. Use one of: {valid}"
            )
            return

        if not self.image_path:
            self.get_logger().error(
                "Parameter image_path is empty. Pass an image path with -p image_path:=..."
            )
            return

        path = Path(self.image_path).expanduser()
        if not path.exists():
            self.get_logger().error(f"Image path does not exist: {path}")
            return

        image = cv2.imread(str(path))
        if image is None:
            self.get_logger().error(f"OpenCV could not read image: {path}")
            return

        mask = self._build_color_mask(image, self.target_color)
        mask = self._clean_mask(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        image_area = image.shape[0] * image.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            if width == 0 or height == 0:
                continue

            aspect_ratio = width / float(height)
            area_ratio = area / float(image_area)

            # These broad filters remove tiny paint fragments and very large road regions.
            if 0.45 <= aspect_ratio <= 4.5 and area_ratio <= 0.08:
                candidates.append((x, y, width, height, area))

        self.detected_count = len(candidates)
        self.summary = (
            f"Detected {self.detected_count} likely {self.target_color} cars "
            f"in {path.name}"
        )

        self.get_logger().info(self.summary)
        self.get_logger().info(
            "This is a color-segmentation estimate, not a trained car detector."
        )

    def _build_color_mask(self, image, target_color):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if target_color == 'red':
            lower_red_a = (0, 70, 50)
            upper_red_a = (10, 255, 255)
            lower_red_b = (170, 70, 50)
            upper_red_b = (180, 255, 255)
            mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)
            mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
            return cv2.bitwise_or(mask_a, mask_b)

        if target_color == 'white':
            lower_white = (0, 0, 150)
            upper_white = (180, 80, 255)
            return cv2.inRange(hsv, lower_white, upper_white)

        lower_black = (0, 0, 0)
        upper_black = (180, 255, 85)
        return cv2.inRange(hsv, lower_black, upper_black)

    def _clean_mask(self, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

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
