#pragma once

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GoToPose : public rclcpp::Node
{
public:
  GoToPose();
  void send_goal(const double x, const double y, const double yaw);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_action;

  GoalHandle::SharedPtr active_goal;
};
