#pragma once

#include <memory>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_mission/waypoint_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Patrol : public rclcpp::Node
{
public:
    Patrol();
    void start();

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

private:
    void send_current();
    void advance();
    void retry_or_skip(const std::string &reason);

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_action;
    std::vector<Waypoint> waypoints;
    GoalHandle::SharedPtr active_goal;
    rclcpp::TimerBase::SharedPtr timeout_timer;

    double timeout_in_seconds = 60.0;
    bool timed_out = false;
    size_t index = 0;
    int attempts = 0;
    int max_attempts = 3;
};