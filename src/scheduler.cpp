#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "multi_waypoint_navigator",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("multi_waypoint_navigator");

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

    // Ask the user to enter waypoint names
    std::vector<std::string> selected_waypoints;
    std::cout << "Enter waypoint names separated by spaces (e.g., wp1 wp2 wp3): ";
    std::string input;
    std::getline(std::cin, input);
    std::istringstream iss(input);
    for (std::string name; iss >> name;)
    {
        selected_waypoints.push_back(name);
    }

    if (selected_waypoints.empty())
    {
        RCLCPP_ERROR(logger, "No waypoints selected.");
        rclcpp::shutdown();
        return 1;
    }

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");

    // Create a vector for waypoints
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;

    for (const auto &waypoint_name : selected_waypoints)
    {
        auto it = std::find_if(waypoints.begin(), waypoints.end(), [&](const json &wp) {
            return wp["name"].get<std::string>() == waypoint_name;
        });

        if (it == waypoints.end())
        {
            RCLCPP_ERROR(logger, "Waypoint '%s' not found in the JSON file.", waypoint_name.c_str());
            rclcpp::shutdown();
            return 1;
        }

        const auto &waypoint = *it;

        // Extract position and orientation
        auto const &position = waypoint["position"];
        auto const &orientation = waypoint["orientation"];

        // Define the goal pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = position["x"].get<double>();
        pose.position.y = position["y"].get<double>();
        pose.position.z = position["z"].get<double>();
        pose.orientation.x = orientation["qx"].get<double>();
        pose.orientation.y = orientation["qy"].get<double>();
        pose.orientation.z = orientation["qz"].get<double>();
        pose.orientation.w = orientation["qw"].get<double>();

        cartesian_waypoints.push_back(pose);
    }

    // Plan the Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(cartesian_waypoints, 0.01, 0.0, trajectory);

    if (fraction < 1.0)
    {
        RCLCPP_WARN(logger, "Only %f%% of the path was planned successfully.", fraction * 100.0);
    }

    if (fraction > 0.0)
    {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = trajectory;
        move_group_interface.execute(cartesian_plan);
        RCLCPP_INFO(logger, "Executed the Cartesian path for selected waypoints.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to compute a Cartesian path for the selected waypoints.");
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
