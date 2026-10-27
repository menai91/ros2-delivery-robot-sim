#include <chrono>
#include <cmath>
#include <memory>

#include "nav_mission/go_to_pose.hpp"

#include "rclcpp/executors.hpp" // needed to use rclcpp::spin

// https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_node.html

GoToPose::GoToPose() : Node("go_to_pose")
{
    client_action = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    RCLCPP_INFO(get_logger(), "go_to_pose node started");
}

void GoToPose::send_goal(const double x, const double y, const double yaw)
{
    if (!client_action->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
        return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted, navigating");
            active_goal = goal_handle;
        }
    };

    options.feedback_callback = [this](const GoalHandle::SharedPtr &goal_handle,
                                       const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal invalid");
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "distance_remaining=%.2f m, recoveries=%d",
            feedback->distance_remaining, feedback->number_of_recoveries);
    };

    options.result_callback = [this](const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal aborted, error_code=%d", result.result->error_code);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }
    };

    client_action->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Sent goal: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GoToPose>();
    node->send_goal(0.0, 4.0, 0.0);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}