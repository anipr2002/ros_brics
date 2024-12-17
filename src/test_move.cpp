#include <Eigen/Dense> // For matrix operations
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Intrinsic Camera Matrix
const double fx = 910.194; // Focal length in x
const double fy = 908.144; // Focal length in y
const double cx = 643.118; // Principal point x
const double cy = 356.288; // Principal point y

// Camera-to-Gripper Transformation Matrix (example values; replace with your
// calibration result)
const Eigen::Matrix4d T_camera_to_gripper =
    (Eigen::Matrix4d() << 0.9999252870724523, -0.012037731443124396,
     -0.002124451641494682, 0.02958811242107153, 0.011982585299959362,
     0.9996329152663407, -0.024299225618241268, 0.07718291107342329,
     0.0024161793399979687, 0.02427195372894834, 0.9997024729086033,
     -0.05315831040014736, 0, 0, 0, 1)
        .finished();

// Function to convert pixel coordinates to 3D in camera frame
Eigen::Vector4d pixel_to_camera_frame(int u, int v, double Z) {
  double X = (u - cx) * Z / fx;
  double Y = (v - cy) * Z / fy;
  return Eigen::Vector4d(X, Y, Z, 1.0); // Homogeneous coordinates
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_move");
  auto logger = rclcpp::get_logger("test_move");

  // TF2 listener for transforms
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // --- Step 1: Convert pixel to camera frame ---
  int pixel_x = 382;
  int pixel_y = 186;
  double depth_Z = 0.5; // Replace with real depth value in meters
  Eigen::Vector4d camera_point =
      pixel_to_camera_frame(pixel_x, pixel_y, depth_Z);

  // --- Step 2: Transform to gripper frame ---
  Eigen::Vector4d gripper_point = T_camera_to_gripper * camera_point;

  // --- Step 3: Transform to base frame using TF2 ---
  geometry_msgs::msg::PointStamped point_gripper, point_base;
  point_gripper.header.frame_id = "tool0"; // Gripper frame
  point_gripper.point.x = gripper_point(0);
  point_gripper.point.y = gripper_point(1);
  point_gripper.point.z = gripper_point(2);

  try {
    // Transform the point from gripper to base frame
    auto transform =
        tf_buffer.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    tf2::doTransform(point_gripper, point_base, transform);

    // --- Step 4: Print goal pose ---
    RCLCPP_INFO(logger,
                "Transformed Position (Base Frame): x=%.4f, y=%.4f, z=%.4f",
                point_base.point.x, point_base.point.y, point_base.point.z);

    // Set neutral orientation for logging purposes
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = point_base.point.x;
    goal_pose.position.y = point_base.point.y;
    goal_pose.position.z = point_base.point.z + 0.1; // Slight offset
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;

    RCLCPP_INFO(logger, "Goal Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                goal_pose.orientation.x, goal_pose.orientation.y,
                goal_pose.orientation.z, goal_pose.orientation.w);

  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(logger, "Transform error: %s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}
