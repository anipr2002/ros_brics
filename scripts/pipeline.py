#!/usr/bin/env python3

import sys
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from ros_brics.action import MoveToPose
from ros_brics.msg import DetectionResult
from tf_transformations import quaternion_from_euler


class RobotControlNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.get_logger().info("Robot Control Node started.")

        # Subscribe to detection_result topic
        self.subscription = self.create_subscription(
            DetectionResult, '/detection_result', self.detection_callback, 10
        )

        self.prev_pose = None
        self.timer = None
        self.detection_count = 0

    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feedback_callback):
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False, "Action server not available"

        # Convert degrees to radians
        roll = roll_deg * math.pi / 180
        pitch = pitch_deg * math.pi / 180
        yaw = yaw_deg * math.pi / 180

        # Create the goal message
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z

        # Convert roll, pitch, yaw to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        goal_msg.target_pose.orientation.x = q[0]
        goal_msg.target_pose.orientation.y = q[1]
        goal_msg.target_pose.orientation.z = q[2]
        goal_msg.target_pose.orientation.w = q[3]

        self.get_logger().info(f"Sending goal: {goal_msg.target_pose}")

        # Send the goal and register feedback callback
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by the server.")
            return False, "Goal rejected"

        self.get_logger().info("Goal accepted by the server, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"Goal succeeded: {result.message}")
            return True, result.message
        else:
            self.get_logger().error(f"Goal failed: {result.message}")
            return False, result.message

    def detection_callback(self, msg):
        self.get_logger().info(f"Received detection result: {msg.target_pose}")

        if self.prev_pose is None:
            # Start a timer when receiving the first message
            self.prev_pose = msg.target_pose
            self.detection_count = 1
            self.timer = self.create_timer(5.0, self.check_stability)
        else:
            self.detection_count += 1
            deviation = self.calculate_deviation(self.prev_pose, msg.target_pose)

            self.get_logger().info(f"Deviation: {deviation:.3f} cm")
            self.prev_pose = msg.target_pose

    def calculate_deviation(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2) * 100  # Convert to cm

    def check_stability(self):
        # Stop the timer
        self.destroy_timer(self.timer)

        # If the deviation is within 1 cm, send the pose as a goal
        deviation = self.calculate_deviation(self.prev_pose, self.prev_pose)  # Compare with itself
        if deviation < 1.0:
            self.get_logger().info("Pose is stable, sending to action server...")
            pose = self.prev_pose
            self.send_goal(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                0, 0, 0,  # Assuming the orientation is already in quaternion format
                feedback_callback=self.feedback_callback,
            )
        else:
            self.get_logger().warn("Pose is unstable, not sending to action server.")

        # Reset variables
        self.prev_pose = None
        self.timer = None
        self.detection_count = 0

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f"Action progress: {progress * 100:.1f}%")


class RobotControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Robot Control GUI")
        self.setGeometry(200, 200, 400, 300)

        layout = QVBoxLayout()

        # Single button for initial movement
        self.move_button = QPushButton("Move to Initial Position")
        self.move_button.clicked.connect(self.move_to_initial_position)
        layout.addWidget(self.move_button)

        # Status label
        self.status_label = QLabel("Click the button to start.")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def move_to_initial_position(self):
        self.move_button.setEnabled(False)  # Disable the button
        self.status_label.setText("Moving to Initial Position...")

        def feedback_callback(feedback_msg):
            progress = feedback_msg.feedback.progress
            self.status_label.setText(f"Initial Position: Progress {progress * 100:.1f}%")

        # Move to initial position
        success, message = self.node.send_goal(
            0.1, 0.5, 0.6, -180, 0, 0, feedback_callback=feedback_callback
        )

        if success:
            self.status_label.setText("Initial Position: Goal succeeded!")
        else:
            self.status_label.setText(f"Initial Position: Goal failed! {message}")

        self.move_button.setEnabled(True)  # Re-enable the button


def main():
    rclpy.init()

    # Create the ROS node
    node = RobotControlNode()

    # Start the Qt application
    app = QApplication(sys.argv)
    gui = RobotControlGUI(node)
    gui.show()

    # Run both the GUI and ROS2 event loops
    try:
        app.exec()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
