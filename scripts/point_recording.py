#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import numpy as np
import json
import select
import sys
import termios
import tty
import transforms3d as tf3d

class PointRecordingNode(Node):
    def __init__(self):
        super().__init__('point_recording')

        # TF2 transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Ensure directory exists
        self.config_dir = r"src/ros_brics/config"
        os.makedirs(self.config_dir, exist_ok=True)

        # Path to JSON file
        self.waypoints_path = os.path.join(self.config_dir, 'waypoints.json')

        # Load existing waypoints or initialize
        self.waypoints = self.load_waypoints()

        self.get_logger().info('Point Recording Node initialized')
        self.get_logger().info('Press "1" to save a new waypoint')

        # Timer to periodically check for key press
        self.timer = self.create_timer(0.1, self.check_key_press)

        # Terminal settings for non-blocking key input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def load_waypoints(self):
        """Load existing waypoints from JSON file or return empty list"""
        try:
            if os.path.exists(self.waypoints_path):
                with open(self.waypoints_path, 'r') as f:
                    return json.load(f)
            return []
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return []

    def is_data(self):
        """Check if there's input available"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def check_key_press(self):
        """Check for key press and save point if '1' is pressed"""
        try:
            if self.is_data():
                # Read a single character
                key = sys.stdin.read(1)

                # Save waypoint when '1' is pressed
                if key == '1':
                    self.record_point()
        except Exception as e:
            self.get_logger().error(f'Error checking key press: {e}')

    def record_point(self):
        """Record a new waypoint"""
        try:
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
            quat = [rotation.x, rotation.y, rotation.z, rotation.w]  # Changed order to [qx, qy, qz, qw]

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

            # Prompt user for point name and frame information
            self.get_logger().info('Please enter a name for this waypoint:')
            point_name = sys.stdin.readline().strip()

            if not point_name:
                self.get_logger().warn('Point name cannot be empty')
                return

            parent_frame = 'base_link'  # Specify the parent frame here or prompt user if needed

            # Prepare waypoint data with additional information
            waypoint = {
                'name': point_name,
                'parent_frame': parent_frame,
                'frame': 'tool0',  # Specify the frame here or prompt user if needed
                'position': {
                    'x': translation.x,
                    'y': translation.y,
                    'z': translation.z,
                },
                'orientation': {
                    'qx': quat[0],
                    'qy': quat[1],
                    'qz': quat[2],
                    'qw': quat[3],
                },
                'transform': transform_matrix.tolist()
            }

            # Add to existing waypoints
            self.waypoints.append(waypoint)

            # Save updated waypoints to JSON
            with open(self.waypoints_path, 'w') as f:
                json.dump(self.waypoints, f, indent=4)

            self.get_logger().info(f'Waypoint "{point_name}" saved successfully')

        except Exception as e:
            self.get_logger().error(f'Could not record point: {e}')

    def destroy_node(self):
        """Restore terminal settings when node is destroyed"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PointRecordingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()