#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix
import numpy as np

class ToolToBaseTransformNode(Node):

    def __init__(self):
        super().__init__('tool_to_base_transform_node')

        # Initialize the TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically get and print the transform
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second

    def timer_callback(self):
        try:

            ##### THIS IS THE CORRECT ORDER ########

            transform = self.tf_buffer.lookup_transform(
                'world',  # Target frame
                'tool0',      # Source frame (UR5e's tool frame)
                rclpy.time.Time())  # Use current time

            # Extract translation
            translation = transform.transform.translation
            tx, ty, tz = translation.x, translation.y, translation.z

            # Extract rotation (quaternion)
            rotation = transform.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

            # Convert quaternion to a 4x4 rotation matrix
            rotation_matrix = quaternion_matrix([qx, qy, qz, qw])

            # Insert the translation into the transformation matrix
            T_base_tool = np.eye(4)
            T_base_tool[:3, :3] = rotation_matrix[:3, :3]
            T_base_tool[:3, 3] = [tx, ty, tz]

            matrix_str = "[" + "\n ".join(["[" + ", ".join(f"{value:.6f}" for value in row) + "]" for row in T_base_tool]) + "]"
            self.get_logger().info(f"Transformation matrix: \n{matrix_str}")

            self.get_logger().info(f"Tool to Base Transformation: \n"
                                   f"Translation: ({round(transform.transform.translation.x, 5)}, "
                                   f"{round(transform.transform.translation.y, 5)}, "
                                   f"{round(transform.transform.translation.z, 5)})\n"
                                   f"Rotation: ({
                                       transform.transform.rotation.x}, "
                                   f"{transform.transform.rotation.y}, "
                                   f"{transform.transform.rotation.z}, "
                                   f"{transform.transform.rotation.w})")

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = ToolToBaseTransformNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
