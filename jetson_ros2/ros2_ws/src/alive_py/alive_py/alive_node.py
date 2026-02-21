#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class AliveNode(Node):
    def __init__(self):
        super().__init__('alive_node')

        # Parameter for publish rate
        self.declare_parameter('rate_hz', 1.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0:
            rate_hz = 1.0

        self.pub = self.create_publisher(Bool, '/alive', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(f"Publishing /alive at {rate_hz} Hz")

    def tick(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AliveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
