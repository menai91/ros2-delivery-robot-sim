#pragma once

#include <chrono>
#include <memory>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_navigation
{
    enum ErrorCode : unsigned int
    {
        Success = 0,
        InProgress,
        ActionServerNotReady,
        InvalidNavGoal,
        Cancelled,
        GoalResponseTimeout,

        //Add entries before UnknownError
        UnknownError
    };

    enum class NavStatus
    {
        Idle,
        Running,
        Succeeded,
        Failed,
        Cancelled
    };

    class Navigator
    {
    public:
        explicit Navigator(rclcpp::Node::SharedPtr node, std::chrono::seconds in_goal_response_timeout = std::chrono::seconds(10));

        bool start(const float x, const float y, const float yaw);
        
        NavStatus get_status() const { return status; }
        unsigned int get_error_code() const { return error_code; }

        void cancel();
        bool has_pending_responses() const { return pending_goal_responses > 0 || pending_cancel_responses > 0; }
        void cancel_all_goals();

    private:
        using NavAction = nav2_msgs::action::NavigateToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;
        using CancelResponse = NavAction::Impl::CancelGoalService::Response;

        void set_status(const NavStatus in_status) { status = in_status; }
        void set_error_code(const unsigned int in_error_code) { error_code = in_error_code; }

        void stop_goal_response_timer();
        void send_cancel(const GoalHandle::SharedPtr &handle);

    private:
        rclcpp::Node::SharedPtr node;
        rclcpp_action::Client<NavAction>::SharedPtr client_action;
        rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr goal_handle;

        // used for time out behavior of the navigation
        std::chrono::seconds goal_response_timeout;
        rclcpp::TimerBase::SharedPtr goal_response_timer;

        NavStatus status = NavStatus::Idle;
        unsigned int error_code = 0;
        unsigned int goal_id_counter = 0;

        unsigned int pending_goal_responses = 0;
        unsigned int pending_cancel_responses = 0;
    };

} // namespace amr_navigation