#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char *argv[])
{
    // Initialize rclcpp (ROS Client Library for C++ package)
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "move_program",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_program");

    // Create the MoveIt MoveGroup Interface
    // Update the planning group for UR5e, usually "manipulator"
    moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "ur_manipulator");

    // Define the goal pose
    geometry_msgs::msg::Pose GoalPose;
    // Use the provided quaternion for orientation
    GoalPose.orientation.x = 1;
    GoalPose.orientation.y = 0;
    GoalPose.orientation.z = 0;
    GoalPose.orientation.w = 0;

// (x=0.09257420272627895, y=-0.6782116709819181, z=0.5165066304554246)
    // Set a position; adjust these values as necessary to avoid collisions
    GoalPose.position.x = 0.137348, 
    GoalPose.position.y = 0.499122;
    GoalPose.position.z = 0.688045;

    // Set the pose target
    MoveGroupInterface.setPoseTarget(GoalPose);

    // Create a plan to move the robot to the goal pose
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    auto const outcome = static_cast<bool>(MoveGroupInterface.plan(plan1));

    // Execute the plan
    if (outcome)
    {
        MoveGroupInterface.execute(plan1);
        RCLCPP_INFO(logger, "Plan executed successfully.");
    }
    else
    {
        RCLCPP_ERROR(logger, "We were not able to plan and execute!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
