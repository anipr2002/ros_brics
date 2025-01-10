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

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO(
            os.environ['HOME'] + '/CODE/ROS_WS/ros_ws/src/ros_brics/config/BRICS/trains/InitialPass/InitialPass_control/weights/best.pt')
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

                    # Draw points and connecting line on the annotated frame
                    annotated_frame = self.draw_points(annotated_frame, midpoint, grasp_point1, grasp_point2)

                    self.yolov8_inference.yolov8_inference.append(
                        self.inference_result)
            else:
                camera_subscriber.get_logger().info(f"no_results")

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()