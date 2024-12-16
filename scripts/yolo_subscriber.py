#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_brics.msg import Yolo  # Replace 'ros_brics' with your package name


class YoloSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_subscriber")
        self.subscription_ = self.create_subscription(
            Yolo, "yolo_data", self.callback, 10)  # Topic name: 'yolo_data'
        self.get_logger().info("Yolo Subscriber Node has been started.")

    def callback(self, msg):
        self.get_logger().info(f"Received: is_usable={
            msg.is_usable}, confidence={msg.confidence}")


class YoloSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_subscriber")
        self.subscription = self.create_subscription(
            Yolo,
            "yolo_data",  # Topic name must match the publisher
            self.listener_callback,
            10  # Queue size
        )
        self.get_logger().info("Yolo Subscriber Node has been started.")

    def listener_callback(self, msg):
        # Callback function is triggered whenever a message is received
        self.get_logger().info(f"Received: is_usable={
            msg.is_usable}, confidence={msg.confidence}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped via KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
