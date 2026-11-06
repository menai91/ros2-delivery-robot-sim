#include "amr_autonomy/bt_set_task_outcome.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("SetTaskOutcome"); }
}

namespace amr_autonomy
{
    SetTaskOutcome::SetTaskOutcome(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::TaskQueue> in_task_queue)
        : BT::SyncActionNode(name, config), task_queue(std::move(in_task_queue))
    {
        if (task_queue == nullptr)
        {
            throw BT::RuntimeError("TaskQueue is not valid");
        }
    }

    BT::PortsList SetTaskOutcome::providedPorts()
    {
        return {
            BT::InputPort<amr_autonomy::TaskOutcome>("outcome", "outcome of the task"),
            BT::InputPort<std::string>("reason", "", "any optional string to provide with the outcome"),
        };
    }

    BT::NodeStatus SetTaskOutcome::tick()
    {
        const auto outcome = getInput<amr_autonomy::TaskOutcome>("outcome");
        const auto reason = getInput<std::string>("reason");

        // the port defaults to "", so only log a reason the tree actually gave.
        // TaskQueue logs the outcome itself, this only adds why
        if (reason.has_value() && !reason.value().empty())
        {
            RCLCPP_INFO(logger(), "[%s] Reason: %s", name().c_str(), reason.value().c_str());
        }

        if (!outcome)
        {
            RCLCPP_ERROR(logger(), "[%s] Invalid input port 'outcome': %s", name().c_str(), outcome.error().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (task_queue->set_task_outcome(*outcome))
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
}