#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ros_brics.msg import Yolo  # Replace 'ros_brics' with your package name

import random


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.publisher_ = self.create_publisher(
            Yolo, "yolo_data", 10)  # Topic name: 'yolo_data'
        # Timer with 1-second interval
        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info("Yolo Publisher Node has been started.")

    def publish_message(self):
        msg = Yolo()
        msg.is_usable = random.choice([True, False])  # Random boolean value
        # Random float between 0.0 and 1.0
        msg.confidence = round(random.uniform(0.0, 1.0), 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: is_usable={
            msg.is_usable}, confidence={msg.confidence}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped via KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

