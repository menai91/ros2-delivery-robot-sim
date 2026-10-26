#include "nav_mission/patrol.hpp"

#include <chrono>
#include <cmath>
#include <string>

Patrol::Patrol() : Node("patrol")
{
    const auto path = declare_parameter<std::string>("waypointsfile", "");
    max_attempts = declare_parameter<int>("max_attempts", 3);
    timeout_in_seconds = declare_parameter<double>("waypoint_timeout_second", 60.0);

    if (path.empty())
    {
        throw std::runtime_error("Parameter 'waypointsfile' is required");
    }

    if(max_attempts < 0)
    {
        throw std::runtime_error("Parameter 'max_attempts' is a value below 0");
    }

    if(max_attempts < 0.0)
    {
        throw std::runtime_error("Parameter 'waypoint_timeout_second' is a value below 0.0");
    }

    // throws on a bad file
    waypoints = load_waypoints(path);

    client_action = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints.size());
}

void Patrol::start()
{
    if (!client_action->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
        rclcpp::shutdown();
        return;
    }

    send_current();
}

void Patrol::advance()
{
    attempts = 0;
    ++index;
    send_current();
}

void Patrol::send_current()
{
    if (index >= waypoints.size())
    {
        RCLCPP_INFO(get_logger(), "Patrol complete");
        rclcpp::shutdown();
        return;
    }

    const Waypoint &wp = waypoints[index];

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = wp.x;
    goal.pose.pose.position.y = wp.y;
    goal.pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(wp.yaw / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            retry_or_skip("rejected");
        }
        else
        {
            active_goal = goal_handle;
            timeout_timer = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(timeout_in_seconds * 1000)),
                [this, goal_handle]()
                {
                    // disable the timer
                    timeout_timer->cancel();
                    if (active_goal == goal_handle)
                    {
                        // if it's still the same goal, something is wrong
                        RCLCPP_WARN(get_logger(), "Waypoint %zu timed out after %.0f s, cancelling",
                                    index, timeout_in_seconds);

                        timed_out = true;
                        client_action->async_cancel_goal(goal_handle);
                    }
                });
        }
        
    };

    //TODO: use feedback if needed
    options.feedback_callback;

   constexpr int kGoalOutsideMap = 204;
   constexpr int kGoalOccupied = 206;

    options.result_callback = [this](const GoalHandle::WrappedResult &result)
    {
        if (timeout_timer)
        {
            timeout_timer->cancel();
        }
        
        active_goal.reset();

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {

            RCLCPP_INFO(get_logger(), "Waypoint %zu reached", index);
            advance();
        }
        break;
        // aborted -> cancelled by the server (Nav2)
        case rclcpp_action::ResultCode::ABORTED:
        {
            const int code = result.result->error_code;
            if (code == kGoalOutsideMap || code == kGoalOccupied)
            {
                RCLCPP_ERROR(get_logger(), "Waypoint %zu is invalid (error_code=%d), skipping", index, code);
                advance();
            }
            else
            {
                retry_or_skip("aborted, error_code=" + std::to_string(code));
            }
        }
        break;
        // canceled -> cancelled by the user (client)
        case rclcpp_action::ResultCode::CANCELED:
        {
            if (timed_out)
            {
                retry_or_skip("timeout");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Waypoint %zu canceled, skipping", index);
                advance();
            }
        }
        break;
        default:
        {
            RCLCPP_WARN(get_logger(), "Waypoint %zu ended with code %d, skipping",
                        index, static_cast<int>(result.code));

            advance();
        }
        break;
        }

        timed_out = false;
    };

    RCLCPP_INFO(get_logger(), "Sending waypoint %zu: x=%.2f y=%.2f yaw=%.2f",
                index, wp.x, wp.y, wp.yaw);

    client_action->async_send_goal(goal, options);
}

void Patrol::retry_or_skip(const std::string &reason)
{
    ++attempts;
    if (attempts < max_attempts)
    {
        RCLCPP_WARN(get_logger(), "Waypoint %zu failed (%s), retry %d of %d",
                    index, reason.c_str(), attempts, max_attempts - 1);

        send_current();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Waypoint %zu failed (%s), giving up, skipping",
                     index, reason.c_str());
        advance();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<Patrol>();
        node->start();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("patrol"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}