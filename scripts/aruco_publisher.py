#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # Define the camera matrix and distortion coefficients
        self.camera_matrix = np.array([
            [910.1943, 0.0000, 643.1175],
            [0.0000, 908.1443, 356.2875],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))

        # ROS2 Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.image_pub = self.create_publisher(Image, 'aruco_image', 10)

        self.bridge = CvBridge()

        # ArUco marker setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.parameters)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(gray)

        # Initialize PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = msg.header.frame_id
        pose_array.header.stamp = msg.header.stamp

        if ids is not None:
            # Draw markers and process poses
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.03, self.camera_matrix, self.dist_coeffs
                )
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03
                )

                # Create Pose message
                pose = Pose()
                pose.position.x = tvec[0][0][0]
                pose.position.y = tvec[0][0][1]
                pose.position.z = tvec[0][0][2]
                # Orientation can be derived if needed, for simplicity omitted here
                pose_array.poses.append(pose)

                # Annotate frame with pose info
                pos_text = (
                    f"Marker {ids[i][0]}: x={pose.position.x:.2f}, "
                    f"y={pose.position.y:.2f}, z={pose.position.z:.2f}"
                )
                cv2.putText(
                    frame,
                    pos_text,
                    (10, 30 + 30 * i),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        # Publish poses
        self.pose_pub.publish(pose_array)

        # Publish image
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
