#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters
        self.declare_parameter('camera_id', 6)  # Default camera ID is 0
        self.declare_parameter('frame_rate', 30.0)  # Default frame rate is 30 FPS

        # Get parameters
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value

        # Initialize VideoCapture
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera with ID {self.camera_id}')
            raise RuntimeError(f'Cannot open camera {self.camera_id}')

        self.get_logger().info(f'Camera {self.camera_id} opened successfully')

        # Create a publisher
        self.image_pub = self.create_publisher(Image, 'camera/camera2/image_raw', 10)

        # Create a timer for publishing
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frame)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame from camera')
            return

        try:
            # Convert the frame to a ROS Image message and publish
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error converting frame: {e}')

    def destroy_node(self):
        # Release the VideoCapture object when shutting down
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Camera Publisher Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
