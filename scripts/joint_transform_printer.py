#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class JointTransformPrinter(Node):
    def __init__(self):
        super().__init__('joint_transform_printer')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Print joint states
        for i in range(len(msg.name)):
            self.get_logger().info(f'Joint: {msg.name[i]}, Position: {msg.position[i]}')

        # Optionally publish transforms here if needed
        # Example for creating and broadcasting transforms can be added
        # Example for creating and broadcasting transforms can be added
                # Example for broadcasting transforms (add this in listener_callback)
        broadcaster = TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame
        t.child_frame_id = msg.name[i]   # Child frame (joint name)

        # Set translation and rotation based on joint position (simplified)
        t.transform.translation.x = 0  # Set based on actual kinematics
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.w = 1.0     # Set based on actual kinematics

        broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    joint_transform_printer = JointTransformPrinter()
    rclpy.spin(joint_transform_printer)
    joint_transform_printer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
