#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state')

        # Get the initial state from parameters
        self.state = self.declare_parameter(
            'initial_state', 'Idle').get_parameter_value().string_value
        self.get_logger().info(f"Initial state set to: {self.state}")

        # Publisher to announce the robot state
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        # Timer to periodically publish the current state
        self.create_timer(1.0, self.publish_state)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
