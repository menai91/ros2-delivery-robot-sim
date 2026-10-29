#include "amr_navigation/navigator.hpp"

#include <cmath> // for std::sin && std::cos

namespace amr_navigation
{
    Navigator::Navigator(rclcpp::Node::SharedPtr in_node, std::chrono::seconds in_goal_response_timeout /*= std::chrono::seconds(10)*/)
        : node(in_node), goal_response_timeout(in_goal_response_timeout)
    {
        client_action = rclcpp_action::create_client<NavAction>(node, "navigate_to_pose");

        RCLCPP_INFO(node->get_logger(), "Waiting for 'navigate_to_pose' action server...");

        if (client_action->wait_for_action_server(std::chrono::seconds(15)))
        {
            RCLCPP_INFO(node->get_logger(), "Connected to 'navigate_to_pose' action server");
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "'navigate_to_pose' action server not available after 15s, goals will fail until Nav2 is up");
        }
    }

    bool Navigator::start(const float x, const float y, const float yaw)
    {
        if (!client_action->action_server_is_ready())
        {
            /*Mostly to handle the case where the server node crashes. cancel() increments the counter,
            stops the timer and cancels any running ongoing goals (or goals that have yet to be sent from the action server)*/
            cancel();

            RCLCPP_ERROR(node->get_logger(), "Cannot send goal (%.2f, %.2f, %.2f): 'navigate_to_pose' action server is not ready", x, y, yaw);
            set_error_code(ErrorCode::ActionServerNotReady);
            set_status(NavStatus::Failed);
            return false;
        }

        // cancel current ongoing goal
        cancel();

        const unsigned int current_goal_id = goal_id_counter;

        /*
        Start a timer in order to fail the goal if Nav2 does not accept or reject it within goal_response_timeout
        or else the Navigator will be stuck in the NavStatus::Running status until we start another navigation node
        or cancel it ourselves*/
        goal_response_timer = node->create_wall_timer(goal_response_timeout, [this, current_goal_id]()
        {
            if(current_goal_id != goal_id_counter)
            {
                return;
            }

            RCLCPP_ERROR(node->get_logger(), "No response from Nav2 for goal #%u within %lds, giving up",
                         current_goal_id, static_cast<long>(goal_response_timeout.count()));

            cancel();
            set_error_code(ErrorCode::GoalResponseTimeout);
            set_status(NavStatus::Failed);
        });

        NavAction::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

        rclcpp_action::Client<NavAction>::SendGoalOptions options;
        options.goal_response_callback = [this, current_goal_id](const GoalHandle::SharedPtr &handle)
        {
            --pending_goal_responses;

            if (current_goal_id != goal_id_counter)
            {
                //for failed goal, handle will be null
                if (handle != nullptr)
                {
                    RCLCPP_INFO(node->get_logger(), "Nav2 accepted stale goal #%u, cancelling it", current_goal_id);
                    send_cancel(handle);
                }

                return;
            }

            stop_goal_response_timer();

            if (!handle)
            {
                RCLCPP_ERROR(node->get_logger(), "Goal #%u was rejected by Nav2", current_goal_id);
                set_error_code(ErrorCode::InvalidNavGoal);
                set_status(NavStatus::Failed);
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Goal #%u accepted, navigating", current_goal_id);
                goal_handle = handle;
            }
        };

        int last_recoveries = 0;
        options.feedback_callback = [this, current_goal_id, last_recoveries](const GoalHandle::SharedPtr &handle,
                                                               const std::shared_ptr<const NavAction::Feedback> feedback) mutable
        {
            if (current_goal_id != goal_id_counter)
            {
                return;
            }

            if (!handle)
            {
                RCLCPP_ERROR(node->get_logger(), "Goal invalid");
                set_error_code(ErrorCode::InvalidNavGoal);
                set_status(NavStatus::Failed);
                return;
            }

            if (!feedback)
            {
                return;
            }

            if (feedback->number_of_recoveries > last_recoveries)
            {
                last_recoveries = feedback->number_of_recoveries;
                RCLCPP_WARN(node->get_logger(), "Nav2 is running a recovery behavior for goal #%u (%d so far)",
                            current_goal_id, last_recoveries);
            }

            RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Goal #%u: %.2f m remaining",
                                  current_goal_id, feedback->distance_remaining);
        };

        options.result_callback = [this, current_goal_id](const GoalHandle::WrappedResult &result)
        {
            if (current_goal_id != goal_id_counter)
            {
                RCLCPP_DEBUG(node->get_logger(), "Ignoring result for stale goal #%u", current_goal_id);
                return;
            }

            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_INFO(node->get_logger(), "Goal #%u reached", current_goal_id);
                set_error_code(ErrorCode::Success);
                set_status(NavStatus::Succeeded);
            }
            break;

            case rclcpp_action::ResultCode::ABORTED:
            {
                RCLCPP_ERROR(node->get_logger(), "Goal #%u aborted by Nav2, error_code=%d: %s",
                             current_goal_id, result.result->error_code, result.result->error_msg.c_str());

                set_error_code(result.result->error_code);
                set_status(NavStatus::Failed);
            }
            break;

            case rclcpp_action::ResultCode::CANCELED:
            {
                /*Normally wont be reachable due to the goal_count check above.
                The correct flow would be cancel() is called, this increments the goal count
                and fires an async_cancel_goal(). By the time the result callback is called
                the goal counters do not match, so it will early return. */
                RCLCPP_WARN(node->get_logger(), "Goal #%u cancelled", current_goal_id);
                set_error_code(ErrorCode::Cancelled);
                set_status(NavStatus::Cancelled);
            }
            break;

            default:
            {
                RCLCPP_ERROR(node->get_logger(), "Goal #%u finished with unknown result code %d", 
                             current_goal_id, static_cast<int>(result.code));

                set_error_code(ErrorCode::UnknownError);
                set_status(NavStatus::Failed);
            }
            break;
            }

            goal_handle.reset();
        };

        RCLCPP_INFO(node->get_logger(), "Sending goal #%u: x=%.2f, y=%.2f, yaw=%.2f", current_goal_id, x, y, yaw);

        set_error_code(ErrorCode::InProgress);
        set_status(NavStatus::Running);
        ++pending_goal_responses;
        client_action->async_send_goal(goal, options);

        return true;
    }

    void Navigator::cancel()
    {
        /*increment to catch an edge case where we have already called start and waiting for the response 
        callback but we call cancel. We do not have a valid goal_handle, so we cant stop it. So increment
        the counter so that when the goal handle arrives at goal_response_callback we can cancel the goal */
        ++goal_id_counter;
        stop_goal_response_timer();

        if (status == NavStatus::Running)
        {
            if (goal_handle != nullptr)
            {
                RCLCPP_INFO(node->get_logger(), "Cancelling active goal");
                send_cancel(goal_handle);
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Cancel requested before Nav2 responded, goal will be cancelled if accepted");
            }

            set_error_code(ErrorCode::Cancelled);
            set_status(NavStatus::Cancelled);
        }

        goal_handle.reset();
    }

    void Navigator::cancel_all_goals()
    {
        // Cancels goals from every client on this server, not only ours (e.g. RViz goals too)
        ++pending_cancel_responses;

        client_action->async_cancel_all_goals([this](const std::shared_ptr<NavAction::Impl::CancelGoalService::Response> response)
            {
                --pending_cancel_responses;
                if (response && response->return_code != CancelResponse::ERROR_NONE)
                {
                    RCLCPP_WARN(node->get_logger(), "Nav2 refused cancel-all request, return_code=%d", response->return_code);
                }
            });
    }

    void Navigator::stop_goal_response_timer()
    {
        if (goal_response_timer)
        {
            goal_response_timer->cancel();
            goal_response_timer.reset();
        }
    }

    void Navigator::send_cancel(const GoalHandle::SharedPtr &handle)
    {
        ++pending_cancel_responses;

        client_action->async_cancel_goal(handle, [this](const std::shared_ptr<NavAction::Impl::CancelGoalService::Response> response)
            {
                --pending_cancel_responses;
                
                // UNKNOWN_GOAL_ID / GOAL_TERMINATED just mean the goal already finished
                if (response && response->return_code == CancelResponse::ERROR_REJECTED)
                {
                    RCLCPP_WARN(node->get_logger(), "Nav2 rejected the cancel request, goal may still be running");
                }
            });
    }
} // namespace amr_navigation