#!/usr/bin/env python3
from ultralytics import YOLO
import os
import copy
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_brics.msg import InferenceResult
from ros_brics.msg import Yolov8Inference
import tf2_ros
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import math 
from geometry_msgs.msg import Pose
from ros_brics.msg import DetectionResult 

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO('/root/ros_ws/src/ros_brics/config/trains/InitialPass/InitialPass_control/weights/best.pt')
        self.yolov8_inference = Yolov8Inference()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.camera_callback,
            10)
        self.subscription
        self.yolov8_pub = self.create_publisher(
            Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.detection_pub = self.create_publisher(DetectionResult, "/detection_result", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = np.array([
            [609.62548, 0.0000, 325.14709],
            [0.0000, 609.64825, 237.64407],
            [0, 0, 1]
        ])

    def calculate_center_point(self, coordinates):
        points = np.array(coordinates).reshape(4, 2)
        center_x = np.mean(points[:, 0])
        center_y = np.mean(points[:, 1])
        return [center_x, center_y]

    def calculate_grasp_points(self, coordinates, offset_ratio=0.3):
        """
        Calculate two grasp points on either side of the midpoint.
        offset_ratio: determines how far from the center the grasp points will be (as a ratio of the brick's length)
        """
        points = np.array(coordinates).reshape(4, 2)

        # Calculate midpoint
        midpoint = self.calculate_center_point(coordinates)

        # Find the longer side of the rectangle to determine orientation
        # First, calculate the lengths of adjacent sides
        side1 = np.linalg.norm(points[0] - points[1])
        side2 = np.linalg.norm(points[1] - points[2])

        # Determine which pair of points to use based on the longer side
        if side1 < side2:
            # Use points 0 and 1 for direction vector
            direction = points[1] - points[0]
        else:
            # Use points 1 and 2 for direction vector
            direction = points[2] - points[1]

        # Normalize direction vector
        direction = direction / np.linalg.norm(direction)

        # Calculate grasp points on either side of midpoint
        # Use the longer dimension for offset calculation
        offset_distance = max(side1, side2) * offset_ratio

        grasp_point1 = np.array(midpoint) + direction * offset_distance
        grasp_point2 = np.array(midpoint) - direction * offset_distance

        return grasp_point1.tolist(), grasp_point2.tolist()

    def draw_points(self, image, midpoint, grasp_point1, grasp_point2):
        # Draw midpoint (green)
        cv2.circle(image,
                  (int(midpoint[0]), int(midpoint[1])),
                  5, (0, 255, 0), -1)

        # Draw grasp points (blue)
        cv2.circle(image,
                  (int(grasp_point1[0]), int(grasp_point1[1])),
                  5, (255, 0, 0), -1)
        cv2.circle(image,
                  (int(grasp_point2[0]), int(grasp_point2[1])),
                  5, (255, 0, 0), -1)

        # Draw lines connecting the points
        cv2.line(image,
                (int(grasp_point1[0]), int(grasp_point1[1])),
                (int(grasp_point2[0]), int(grasp_point2[1])),
                (0, 0, 255), 2)

        return image

    def calculate_robot(self, center_point):
        """
        Calculate the transformation matrix to move the robot to the YOLO-detected center point.
        Parameters:
            center_point (list): The [x, y] center point from YOLO bounding box in the camera frame.
        """
        T_cam_tool = np.array([
            [1, 0, 0, -0.031],
            [0, 1, 0, -0.06],
            [0, 0, 1, 0.069],
            [0, 0, 0, 1]
        ])

        K_inv = np.linalg.inv(self.camera_matrix)

        try:
            # Lookup transform from world to tool0
            transform = self.tf_buffer.lookup_transform(
                'world',
                'tool0',
                rclpy.time.Time())

            # Extract translation and rotation
            translation = transform.transform.translation
            tx, ty, tz = translation.x, translation.y, translation.z  # Get current position
            rotation = transform.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

            # Convert quaternion to a 4x4 rotation matrix
            rotation_matrix = quaternion_matrix([qx, qy, qz, qw])
            T_base_tool = np.eye(4)
            T_base_tool[:3, :3] = rotation_matrix[:3, :3]
            T_base_tool[:3, 3] = [tx, ty, tz]

            # Compute camera-to-base transformation
            T_base_cam = T_base_tool @ np.linalg.inv(T_cam_tool)

            pixel_coords = np.array([center_point[0], center_point[1], 1])
            # Back-project to 3D
            depth = 0.5
            X_c, Y_c, Z_c = depth * (K_inv @ pixel_coords)

            # Use the center point's x and y, and the tool's current z as the target in the camera frame
            T_target_cam = np.eye(4)
            T_target_cam[:3, 3] = [X_c, Y_c, tz]  # Use current z

            # Compute the final target pose in the robot's base frame
            T_base_target = T_base_cam @ T_target_cam

            target_pose = Pose()
            target_pose.position.x = T_base_target[0, 3]
            target_pose.position.y = T_base_target[1, 3]
            target_pose.position.z = T_base_target[2, 3]

            # Extract orientation from the rotation matrix
            target_orientation = quaternion_from_matrix(T_base_target)
            target_pose.orientation.x = target_orientation[0]
            target_pose.orientation.y = target_orientation[1]
            target_pose.orientation.z = target_orientation[2]
            target_pose.orientation.w = target_orientation[3]

            # Publish the result as DetectionResult
            # Log the resulting transformation matrix
            matrix_str = "[" + "\n ".join(["[" + ", ".join(f"{value:.6f}" for value in row) + "]" for row in T_base_target]) + "]"
            self.get_logger().info(f"Transformation matrix to YOLO center with current z: \n{matrix_str}")

            return target_pose
        
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")


    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        
        results = self.model(img, conf=0.90)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        annotated_frame = results[0].plot()

        for r in results:
            if (r.obb is not None):
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy()
                    c = box.cls
                    confidence = float(box.conf[0])
                    self.inference_result.class_name = self.model.names[int(c)]

                    # Reshape coordinates and store them
                    a = b.reshape(1, 8)
                    coordinates = a[0].tolist()
                    self.inference_result.coordinates = copy.copy(coordinates)

                    # Calculate midpoint and grasp points
                    midpoint = self.calculate_center_point(coordinates)

                    
                    grasp_point1, grasp_point2 = self.calculate_grasp_points(coordinates)

                    # Store points in the message
                    self.inference_result.midpoint = midpoint
                    self.inference_result.grasp_point1 = grasp_point1  # You'll need to add these fields to your msg file
                    self.inference_result.grasp_point2 = grasp_point2

                    dx = grasp_point1[0] - grasp_point2[0]
                    dy = grasp_point1[1] - grasp_point2[1] 

                    angle_radians = math.atan2(dy, dx)
                    angle_degrees = math.degrees(angle_radians)
                    self.get_logger().info(f"Angle in radians: {angle_radians}")
                    self.get_logger().info(f"Angle in degrees: {angle_degrees}")

                    target_pose = self.calculate_robot(midpoint)

                    detection_msg = DetectionResult()
                    detection_msg.class_name = self.inference_result.class_name
                    detection_msg.confidence = confidence
                    detection_msg.target_pose = target_pose
                    detection_msg.gripper_rotate = angle_degrees
                    
                    self.detection_pub.publish(detection_msg)

                    # Draw points and connecting line on the annotated frame
                    annotated_frame = self.draw_points(annotated_frame, midpoint, grasp_point1, grasp_point2)

                    self.yolov8_inference.yolov8_inference.append(
                        self.inference_result)
            else:
                camera_subscriber.get_logger().info(f"no_results")

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        height, width = annotated_frame.shape[:2]
        cv2.circle(annotated_frame, (width // 2, height // 2), 5, (0, 0, 0), -1)

        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
