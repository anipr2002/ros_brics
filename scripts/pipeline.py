#!/usr/bin/env python3
import sys
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from ros_brics.action import MoveToPose
from ros_brics.msg import DetectionResult
from tf_transformations import quaternion_from_euler
from threading import Thread, Event
from robotiq_gripper_ros2.srv import GripperCommand
from robotiq_gripper_ros2 import robotiq_gripper


class RobotControlNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.get_logger().info("Robot Control Node started.")

        self.prev_pose = None
        self.detection_count = 0
        self.stable_pose = None
        self.stable_rotation = None
        self.goal_pose = None  # Class variable to store the goal pose
        self.subscription = None  # Track the subscription to allow its cleanup
        self.timer = None  # Track the timer to destroy it properly
        self.listening_event = Event()  # Event to synchronize listening process

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect("192.168.1.102", 63352)

        self.gripper_min = 0
        self.gripper_max = 255
        

    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feedback_callback):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False, "Action server not available"

        roll = roll_deg * math.pi / 180
        pitch = pitch_deg * math.pi / 180
        yaw = yaw_deg * math.pi / 180

        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        goal_msg.target_pose.orientation.x = q[0]
        goal_msg.target_pose.orientation.y = q[1]
        goal_msg.target_pose.orientation.z = q[2]
        goal_msg.target_pose.orientation.w = q[3]

        self.get_logger().info(f"Sending goal: {goal_msg.target_pose}")

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

    def start_listening(self):
        self.subscription = self.create_subscription(
            DetectionResult, '/detection_result', self.detection_callback, 10
        )
        self.get_logger().info("Started listening to /detection_result.")

        # Clear the event to block the main thread
        self.listening_event.clear()

        # Set a timer to stop listening after 5 seconds
        self.timer = self.create_timer(5.0, self.stop_listening)

    def stop_listening(self):
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None

        if self.stable_pose and self.calculate_deviation(self.prev_pose, self.stable_pose) < 1.0:
            self.goal_pose = [self.stable_pose, -self.stable_rotation]
            self.get_logger().info(f"Goal pose set: {self.goal_pose}")
        else:
            self.get_logger().warn("No stable pose found within 5 seconds.")

        # Signal that listening has stopped
        self.listening_event.set()

    def detection_callback(self, msg):
        self.get_logger().info(f"Received detection result: {msg.target_pose}")
        
        if self.prev_pose is None:
            self.prev_pose = msg.target_pose
            self.detection_count = 1
            self.stable_pose = msg.target_pose
            self.stable_rotation = msg.gripper_rotate
        else:
            deviation = self.calculate_deviation(self.prev_pose, msg.target_pose)
            self.get_logger().info(f"Deviation: {deviation:.3f} cm")

            if deviation < 1.0:
                self.stable_pose = msg.target_pose
                self.stable_rotation = msg.gripper_rotate
            self.prev_pose = msg.target_pose
            self.detection_count += 1

    def calculate_deviation(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2) * 100

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f"Action progress: {progress * 100:.1f}%")

    def activate_gripper(self):
        if self.gripper.is_active:
            self.get_logger().info(f"Gripper is already active, continuing...")
        else:
            self.gripper.activate()
            self.gripper_min = self.gripper.get_min_position()
            self.gripper_max = self.gripper.get_max_position()

    def control_gripper(self, position, speed=255, force=255):
        self.activate_gripper()
        self.gripper.move_and_wait_for_pos(position, speed, force)
        self.get_logger().info(f"Gripper position : {self.gripper.get_current_position()}")
        
        
def run_gum_spinner(message):
    """Run a gum spinner to display a message."""
    subprocess.run(["gum", "spin", "--title", message, "sleep", "2"])


def run_gum_choice(prompt, options):
    """Run a gum choose to display options and get a selection."""
    result = subprocess.run(
        ["gum", "choose"] + options, stdout=subprocess.PIPE, text=True
    )
    return result.stdout.strip()



def main():
    rclpy.init()

    node = RobotControlNode()

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    while True:
        choice = run_gum_choice("Select an Option:", [
            "1. Move to Initial Position",
            "2. Start Listening to Detection Results",
            "3. Send Goal Pose",
            "4. Open Gripper",
            "5. Close Gripper",
            "6. Move Z-axis Down to grip",
            "7. Exit",
        ])

        if choice.startswith("1"):
            run_gum_spinner("Moving to Initial Position")

            def feedback_callback(feedback_msg):
                progress = feedback_msg.feedback.progress
                print(f"[Feedback] Progress: {progress * 100:.1f}%")

            success, message = node.send_goal(0.1, 0.5, 0.6, -180, 0, 0, feedback_callback=feedback_callback)

            if success:
                print("Goal succeeded!")
            else:
                print(f"Goal failed: {message}")

        elif choice.startswith("2"):
            print("Listening for detection results...")
            node.start_listening()

            # Wait until listening has stopped
            node.listening_event.wait()

        elif choice.startswith("3"):
            if node.goal_pose:
                pose = node.goal_pose[0]
                rotation = node.goal_pose[1]
                run_gum_spinner("Sending goal pose to action server")
                node.get_logger().info(f"Rotation : {rotation}")
                def feedback_callback(feedback_msg):
                    progress = feedback_msg.feedback.progress
                    print(f"[Feedback] Progress: {progress * 100:.1f}%")

                success, message = node.send_goal(
                    pose.position.x - 0.06,
                    pose.position.y + 0.14,
                    0.3,
                    -180, 0, rotation, 
                    feedback_callback=feedback_callback
                )

                if success:
                    print("Goal pose sent successfully!")
                else:
                    print(f"Failed to send goal pose: {message}")
            else:
                print("No goal pose available. Please listen to detection results first.")
        
        elif choice.startswith("4"):
            print("Opening gripper...")
            success = node.control_gripper(position=0)
        
        elif choice.startswith("5"):
            print("Closing gripper...")
            success = node.control_gripper(position=255)

        elif choice.startswith("6"):
            print("Moving Z-axis to grip...")
            if node.goal_pose:
                pose = node.goal_pose[0]
                rotation = node.goal_pose[1]
                run_gum_spinner("Moving Z-axis down")

                def feedback_callback(feedback_msg):
                    progress = feedback_msg.feedback.progress
                    print(f"[Feedback] Progress: {progress * 100:.1f}%")

                success, message = node.send_goal(
                    pose.position.x - 0.07,
                    pose.position.y + 0.14,
                    0.24,  # Move 5 cm down
                    -180, 0, rotation ,
                    feedback_callback=feedback_callback
                )

                if success:
                    print("Z-axis moved down successfully!")
                else:
                    print(f"Failed to move Z-axis: {message}")
            else:
                print("No current pose available. Please listen to detection results first.")

        elif choice.startswith("7"):
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please try again.")

    node.get_logger().info("Shutting down...")
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()




if __name__ == "__main__":
    main()
