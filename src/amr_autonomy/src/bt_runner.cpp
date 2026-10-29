#include "amr_autonomy/register_nodes.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

#include <atomic>
#include <csignal>
#include <iostream>

namespace
{
    // Set by Ctrl-C. A signal handler may only do async-signal-safe work, so it just sets a flag.
    std::atomic<bool> stop_requested{false};
    void on_sigint(int) { stop_requested = true; }

    // How long to wait for outstanding Nav2 replies on exit, and for the cancel-all fallback.
    const auto pending_response_timeout = std::chrono::seconds(3);
    const auto cancel_all_timeout = std::chrono::seconds(1);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_behavior_tree.xml>" << std::endl;
        return 1;
    }

    // No rclcpp signal handler: it would shut the context down on Ctrl-C, before we can cancel goals.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

    rclcpp::NodeOptions node_options;
    // override param to use gazebo's sim_time
    node_options.parameter_overrides(
        {{"use_sim_time", true}});

    auto node = std::make_shared<rclcpp::Node>("bt_runner", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto navigator = std::make_shared<amr_navigation::Navigator>(node);

    BT::BehaviorTreeFactory factory;
    amr_autonomy::register_nodes(factory, navigator);

    BT::Tree tree;

    BT::NodeStatus status = BT::NodeStatus::IDLE;

    try
    {
        tree = factory.createTreeFromFile(argv[1]);
        RCLCPP_INFO(node->get_logger(), "Loaded behavior tree from '%s'", argv[1]);

        const auto period = std::chrono::milliseconds(10);
        auto next_tick = std::chrono::steady_clock::now();
        status = BT::NodeStatus::RUNNING;

        while (!stop_requested && rclcpp::ok() && status == BT::NodeStatus::RUNNING)
        {
            // tick the Behavior tree
            status = tree.tickOnce();

            /*spin_some() is responsible for triggering all the callback functions registered in the node
            ex. SendGoalOptions, create_wall_timer
            */
            executor.spin_some();
            next_tick += period;
            std::this_thread::sleep_until(next_tick);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Behavior tree error: %s", e.what());
    }

    if (stop_requested)
    {
        RCLCPP_INFO(node->get_logger(), "Ctrl-C received, stopping behavior tree");
    }
    else if (status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Behavior tree finished with SUCCESS");
    }
    else if (status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_ERROR(node->get_logger(), "Behavior tree finished with FAILURE");
    }
    else if (!rclcpp::ok())
    {
        RCLCPP_WARN(node->get_logger(), "ROS context shut down, stopping behavior tree");
    }

    tree.haltTree();

    // Keep spinning until Nav2 has answered everything we sent, so a late goal accept
    // reaches the stale branch and gets cancelled instead of driving with nobody listening.
    auto wait_for_pending_responses = [&](std::chrono::milliseconds limit)
    {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        while (navigator->has_pending_responses() && std::chrono::steady_clock::now() < deadline)
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    wait_for_pending_responses(pending_response_timeout);

    if (navigator->has_pending_responses())
    {
        RCLCPP_WARN(node->get_logger(), "Nav2 replies still pending at exit, cancelling all goals");
        navigator->cancel_all_goals();
        wait_for_pending_responses(cancel_all_timeout);

        if (navigator->has_pending_responses())
        {
            RCLCPP_ERROR(node->get_logger(), "Nav2 still has not replied, exiting anyway. The robot may still be executing a goal");
        }
    }

    rclcpp::shutdown();
    return 0;
}
