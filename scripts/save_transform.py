#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import numpy as np
import transforms3d as tf3d
import datetime
import json
import select
import sys
import termios
import tty
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UR5eTCPTransformNode(Node):
    def __init__(self):
        super().__init__('save_transform')

        # TF2 transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RealSense image subscriber
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Timer to periodically check for key press
        self.timer = self.create_timer(0.1, self.check_key_press)

        # Ensure directories exist
        self.config_dir = r"src/ros_brics/config"
        self.transforms_dir = os.path.join(self.config_dir, 'transforms')
        self.images_dir = os.path.join(self.config_dir, 'images')
        os.makedirs(self.transforms_dir, exist_ok=True)
        os.makedirs(self.images_dir, exist_ok=True)

        # Path to JSON file
        self.tf_json_path = os.path.join(self.config_dir, 'tf2.json')

        # Load existing transforms or initialize
        self.transforms = self.load_transforms()

        # Counter for transforms
        self.transform_count = len(self.transforms)

        self.get_logger().info('UR5e TCP Transform Node initialized')
        self.get_logger().info('Press "1" to save current TCP transform and image')
        self.get_logger().info(
            f'Transforms will be saved to {self.transforms_dir}')
        self.get_logger().info(f'Images will be saved to {self.images_dir}')

        # Terminal settings for non-blocking key input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def image_callback(self, msg):
        """Callback to store the latest image from RealSense"""
        try:
            # Convert ROS image message to OpenCV image
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def load_transforms(self):
        """Load existing transforms from JSON file or return empty list"""
        try:
            if os.path.exists(self.tf_json_path):
                with open(self.tf_json_path, 'r') as f:
                    return json.load(f)
            return []
        except Exception as e:
            self.get_logger().error(f'Error loading transforms: {e}')
            return []

    def is_data(self):
        """Check if there's input available"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def check_key_press(self):
        """Check for key press and save transform if '1' is pressed"""
        try:
            if self.is_data():
                # Read a single character
                key = sys.stdin.read(1)

                # Save transform when '1' is pressed
                if key == '1':
                    self.save_transform()
        except Exception as e:
            self.get_logger().error(f'Error checking key press: {e}')

    def save_transform(self):
        """Save current TCP transform to NPZ file and update JSON"""
        try:
            # Ensure we have both transform and image
            if self.latest_image is None:
                self.get_logger().warn('No image available to save')
                return

            # Look up the transform from base_link to tool0
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )

            # Extract translation
            translation = transform.transform.translation

            # Extract rotation (as quaternion)
            rotation = transform.transform.rotation
            quat = [rotation.w, rotation.x, rotation.y, rotation.z]

            # Convert quaternion to rotation matrix
            rot_matrix = tf3d.quaternions.quat2mat(quat)

            # Create 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rot_matrix
            transform_matrix[:3, 3] = [
                translation.x,
                translation.y,
                translation.z
            ]

            # Generate unique filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            transform_filename = f'TBase2EE_{self.transform_count:03d}.npz'
            transform_filepath = os.path.join(
                self.transforms_dir, transform_filename)

            # Generate image filename
            image_filename = f'color_image_{self.transform_count:03d}.png'
            image_filepath = os.path.join(self.images_dir, image_filename)

            # Save transform matrix to NPZ file
            np.savez(transform_filepath,
                     arr_0=transform_matrix
                     )

            # Save image
            cv2.imwrite(image_filepath, self.latest_image)

            # Prepare transform data for JSON
            transform_entry = {
                'name': transform_filename,
                'image': image_filename,
                'transform': transform_matrix.tolist()
            }

            # Add to existing transforms
            self.transforms.append(transform_entry)

            # Save updated transforms to JSON
            with open(self.tf_json_path, 'w') as f:
                json.dump(self.transforms, f, indent=4)

            # Increment transform counter
            self.transform_count += 1

            self.get_logger().info(f'Transform saved: {transform_filename}')
            self.get_logger().info(f'Transform : {transform_matrix}')
            self.get_logger().info(f'Image saved: {image_filename}')
            self.get_logger().info(
                f'Total saved transforms: {len(self.transforms)}')

        except Exception as e:
            self.get_logger().error(f'Could not save transform: {e}')

    def destroy_node(self):
        """Restore terminal settings when node is destroyed"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eTCPTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
