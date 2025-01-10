#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import transforms3d as tf3d

class TFMatrixPrinter(Node):
    def __init__(self):
        super().__init__('joint_transform_printer')

        # TF2 Buffer and Listener to listen for TFs
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically fetch the transformation
        self.timer = self.create_timer(1.0, self.print_transform)

        self.get_logger().info("TF Matrix Printer Node started. Waiting for /tf data...")

    def print_transform(self):
        try:
            # Lookup transform from base_link to tool0
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time()
            )

            # Extract translation
            t = transform.transform.translation
            translation = np.array([t.x, t.y, t.z])

            # Extract rotation quaternion and convert to rotation matrix
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]
            rotation_matrix = tf3d.quaternions.quat2mat([q.x, q.y, q.z, q.wn])  # Ensure correct quaternion order

            # Construct full 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = translation

            # Print the transformation matrix
            self.get_logger().info("Transformation Matrix (base_link -> tool0):")
            self.get_logger().info("\n" + str(transform_matrix))

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TFMatrixPrinter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
