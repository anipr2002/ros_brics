#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np
import transforms3d as tf3d


class TestMovepy(Node):
    def __init__(self):
        super().__init__('test_movepy')

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Intrinsic matrix (calibrated values)
        self.K = np.array([
            [910.1943222874194, 0, 643.1175091984419],
            [0, 908.1443292139583, 356.28754896411783],
            [0, 0, 1]
        ])

        # Camera-to-gripper transformation matrix
        self.T_camera_to_gripper = np.array([
            [0.9999253655821398, -0.012031318750677051, -0.0021238254504012274, 0.029588906409917317],
            [0.0119761994119368, 0.9996330896825083, -0.024295198279658304, 0.07718277005621094],
            [0.002415349471544393, 0.02426794966456775, 0.9997025721213326, -0.053158496141739855],
            [0, 0, 0, 1]
        ])

        # Pixel coordinates from YOLO detection
        self.pixel_x = 382
        self.pixel_y = 186

        # Assume depth at the pixel (replace this with real depth image)
        self.Z = 0.5  # Example depth value in meters

        self.timer = self.create_timer(1.0, self.compute_position)
        self.get_logger().info("Brick Position Node initialized, retrieving TFs...")

    def compute_position(self):
        try:
            # Step 1: Convert pixel coordinates to camera frame
            X_c = (self.pixel_x - self.K[0, 2]) * self.Z / self.K[0, 0]
            Y_c = (self.pixel_y - self.K[1, 2]) * self.Z / self.K[1, 1]
            camera_point = np.array([X_c, Y_c, self.Z, 1])

            self.get_logger().info(f"Camera Frame: X={X_c:.4f}, Y={Y_c:.4f}, Z={self.Z:.4f}")

            # Step 2: Transform to gripper frame
            gripper_point = np.dot(self.T_camera_to_gripper, camera_point)
            self.get_logger().info(f"Gripper Frame: X={gripper_point[0]:.4f}, "
                                   f"Y={gripper_point[1]:.4f}, Z={gripper_point[2]:.4f}")

            # Step 3: Retrieve tool0-to-base_link transform
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to transformation matrix
            T_gripper_to_base = np.eye(4)
            rot_matrix = tf3d.quaternions.quat2mat([rotation.w, rotation.x, rotation.y, rotation.z])
            T_gripper_to_base[:3, :3] = rot_matrix
            T_gripper_to_base[:3, 3] = [translation.x, translation.y, translation.z]

            # Step 4: Transform to base_link frame
            base_point = np.dot(T_gripper_to_base, gripper_point)

            # Log the results
            self.get_logger().info(f"Base Frame: X={base_point[0]:.4f}, "
                                   f"Y={base_point[1]:.4f}, Z={base_point[2]:.4f}")

        except Exception as e:
            self.get_logger().error(f"Error in computing position: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TestMovepy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
