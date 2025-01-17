#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros_brics/action/move_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

class MoveToPoseActionServer : public rclcpp::Node
{
public:
    using MoveToPose = ros_brics::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    MoveToPoseActionServer()
        : Node("move_to_pose_action_server")
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action server started: move_to_pose");
    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request.");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        using namespace moveit::planning_interface;

        // Start the action in a separate thread
        std::thread([this, goal_handle]() {
            auto result = std::make_shared<MoveToPose::Result>();
            auto feedback = std::make_shared<MoveToPose::Feedback>();

            const auto goal = goal_handle->get_goal();

            try
            {
                // Create the MoveGroupInterface using the node's shared pointer
                MoveGroupInterface move_group(shared_from_this(), "ur_manipulator");

                // Set the target pose
                move_group.setPoseTarget(goal->target_pose);

                // Plan and execute the motion
                MoveGroupInterface::Plan plan;
                feedback->progress = 0.5;
                goal_handle->publish_feedback(feedback);

                if (move_group.plan(plan))
                {
                    feedback->progress = 0.8;
                    goal_handle->publish_feedback(feedback);

                    if (move_group.execute(plan))
                    {
                        result->success = true;
                        result->message = "Motion executed successfully.";
                        RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
                    }
                    else
                    {
                        result->success = false;
                        result->message = "Failed to execute motion.";
                        RCLCPP_ERROR(this->get_logger(), "Failed to execute motion.");
                    }
                }
                else
                {
                    result->success = false;
                    result->message = "Failed to plan motion.";
                    RCLCPP_ERROR(this->get_logger(), "Failed to plan motion.");
                }
            }
            catch (const std::exception &e)
            {
                result->success = false;
                result->message = std::string("Exception: ") + e.what();
                RCLCPP_ERROR(this->get_logger(), "Exception during execution: %s", e.what());
            }

            // Publish the result
            goal_handle->succeed(result);
        }).detach();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<MoveToPoseActionServer>();

    // Use a MultiThreadedExecutor to manage threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // Shutdown
    rclcpp::shutdown();
    return 0;
}