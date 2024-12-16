#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <string>

using json = nlohmann::json;

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "waypoint_navigator",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("waypoint_navigator");

    // Load the waypoints JSON file
    const std::string waypoints_file_path = "/home/anipr2002/CODE/ROS_WS/ros_ws/src/ros_brics/config/waypoints.json";
    std::ifstream waypoints_file(waypoints_file_path);
    if (!waypoints_file.is_open())
    {
        RCLCPP_ERROR(logger, "Failed to open waypoints file: %s", waypoints_file_path.c_str());
        rclcpp::shutdown();
        return 1;
    }

    json waypoints;
    try
    {
        waypoints_file >> waypoints;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger, "Error parsing waypoints file: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // List available waypoints
    RCLCPP_INFO(logger, "Available waypoints:");
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        RCLCPP_INFO(logger, "[%zu] %s", i + 1, waypoints[i]["name"].get<std::string>().c_str());
    }

    // Ask the user to select a waypoint
    std::cout << "Enter the number of the waypoint to move to: ";
    size_t choice;
    std::cin >> choice;

    if (choice < 1 || choice > waypoints.size())
    {
        RCLCPP_ERROR(logger, "Invalid choice.");
        rclcpp::shutdown();
        return 1;
    }

    const auto &selected_waypoint = waypoints[choice - 1];

    // Extract position and orientation
    auto const &position = selected_waypoint["position"];
    auto const &orientation = selected_waypoint["orientation"];

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");

    // Define the goal pose
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = position["x"].get<double>();
    goal_pose.position.y = position["y"].get<double>();
    goal_pose.position.z = position["z"].get<double>();

    goal_pose.orientation.x = orientation["qx"].get<double>();
    goal_pose.orientation.y = orientation["qy"].get<double>();
    goal_pose.orientation.z = orientation["qz"].get<double>();
    goal_pose.orientation.w = orientation["qw"].get<double>();

    // Set the pose target
    move_group_interface.setPoseTarget(goal_pose);

    // Plan and execute the movement
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const outcome = static_cast<bool>(move_group_interface.plan(plan));

    if (outcome)
    {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Moved to waypoint: %s", selected_waypoint["name"].get<std::string>().c_str());
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to plan and execute to the selected waypoint.");
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}