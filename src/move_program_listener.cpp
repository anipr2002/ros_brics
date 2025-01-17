#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// Function to convert degrees to radians
double degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// Function to execute the motion plan
void executeMotionPlan(
    const geometry_msgs::msg::Pose &pose,
    rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
{
    using namespace moveit::planning_interface;

    // Create the MoveGroupInterface
    MoveGroupInterface moveGroup(node, "ur_manipulator");

    // Set the pose target
    moveGroup.setPoseTarget(pose);

    // Plan and execute the motion
    MoveGroupInterface::Plan plan;
    if (moveGroup.plan(plan))
    {
        moveGroup.execute(plan);
        RCLCPP_INFO(logger, "Plan executed successfully.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to plan and execute motion!");
    }
}

class PoseListener : public rclcpp::Node
{
public:
    PoseListener()
        : Node("pose_listener")
    {
        // Create a subscription to the "target_pose" topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received target pose");
                try
                {
                    executeMotionPlan(*msg, this->shared_from_this(), this->get_logger());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                }
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PoseListener>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
