#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # Define the camera matrix and distortion coefficients
        self.camera_matrix = np.array([
            [609.62548, 0.0000, 325.14709],
            [0.0000, 609.64825, 237.64407],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))

        # ROS2 Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.image_pub = self.create_publisher(Image, 'aruco_image', 10)

        self.bridge = CvBridge()

        # ArUco marker setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.parameters)
    
    
    def calculate_robot(self, T_marker_cam):
        T_cam_tool = np.array([
            [1, -1, 0, -0.031],
            [0, 1, 0, -0.06],
            [0, 0, 1, 0.069],
            [0, 0, 0, 1]
            ])
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'tool0',
                rclpy.time.Time())

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

            # Compute camera-to-base transformation
            T_base_cam = T_base_tool @ np.linalg.inv(T_cam_tool)

            # Offset transformation: 20 cm above the marker in its z-axis
            T_offset = np.eye(4)
            T_offset[2, 3] = 0.2  # 20 cm along the z-axis

            # Compute marker-to-tool transformation
            T_marker_tool = np.linalg.inv(T_cam_tool) @ T_marker_cam

            # Apply offset to the tool frame
            T_tool_marker = T_offset  # Since we align the z-axis, only the offset is needed

            # Compute the tool pose in the camera frame
            T_tool_cam = T_tool_marker @ np.linalg.inv(T_marker_cam)

            # Compute the final target pose in the robot's base frame
            T_base_tool_target = T_base_cam @ T_tool_cam

            matrix_str = "[" + "\n ".join(["[" + ", ".join(f"{value:.6f}" for value in row) + "]" for row in T_base_tool_target]) + "]"

            self.get_logger().info(f"Transformation matrix: \n{matrix_str}")

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")
        
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
                    corners[i], 0.1, self.camera_matrix, self.dist_coeffs
                )
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1
                )

                # Create Pose message
                pose = Pose()
                pose.position.x = tvec[0][0][0]
                pose.position.y = tvec[0][0][1]
                pose.position.z = tvec[0][0][2]

                pose.orientation.x = rvec[0][0][0]
                pose.orientation.y = rvec[0][0][1]
                pose.orientation.z = rvec[0][0][2]
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
            rotation_matrix, _ = cv2.Rodrigues(rvec[0])

            # Create the transformation matrix
            T_marker_cam = np.eye(4)
            T_marker_cam[:3, :3] = rotation_matrix  # Rotation part
            T_marker_cam[:3, 3] = tvec[0].flatten()  # Translation part
            
            self.calculate_robot(T_marker_cam)
        
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
