#!/usr/bin/env python3

import rclpy
from image_tools_interfaces.srv import CountCarsByColor
from rclpy.node import Node

from image_tools_py.color_car_detector import count_cars_by_color


class ColorCarCounterService(Node):
    """Wait for color-count requests and process images on demand."""

    def __init__(self):
        super().__init__('color_car_counter_service')
        self.declare_parameter('min_area', 450.0)
        self.min_area = float(self.get_parameter('min_area').value)

        self.service = self.create_service(
            CountCarsByColor,
            '/count_cars_by_color',
            self._handle_request,
        )
        self.get_logger().info('Ready: /count_cars_by_color')

    def _handle_request(self, request, response):
        self.get_logger().info(
            f"Request: color={request.target_color}, image={request.image_path}"
        )

        result = count_cars_by_color(
            image_path=request.image_path,
            target_color=request.target_color,
            min_area=self.min_area,
            save_output_image=request.save_output_image,
            output_dir=request.output_dir,
        )

        response.success = result.success
        response.count = result.count
        response.summary = result.summary
        response.output_image_path = result.output_image_path

        if result.success:
            self.get_logger().info(result.summary)
            if result.output_image_path:
                self.get_logger().info(f"Saved annotated image: {result.output_image_path}")
        else:
            self.get_logger().error(result.summary)

        return response


def main():
    rclpy.init()
    node = ColorCarCounterService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
