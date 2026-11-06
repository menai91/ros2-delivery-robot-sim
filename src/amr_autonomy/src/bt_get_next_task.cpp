#include "amr_autonomy/bt_get_next_task.hpp"

#include <string>

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("GetNextTask"); }
}

namespace amr_autonomy
{
    GetNextTask::GetNextTask(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::TaskQueue> in_task_queue)
        : BT::SyncActionNode(name, config), task_queue(in_task_queue)
    {
        if (task_queue == nullptr)
        {
            throw BT::RuntimeError("task_queue is not valid");
        }
    }

    BT::PortsList GetNextTask::providedPorts()
    {
        return {
            BT::OutputPort<double>("pickup_x", "Pickup pose x of the next task (map frame, m)"),
            BT::OutputPort<double>("pickup_y", "Pickup pose y of the next task (map frame, m)"),
            BT::OutputPort<double>("pickup_yaw", "Pickup pose yaw of the next task (rad)"),
            BT::OutputPort<double>("dropoff_x", "Dropoff pose x of the next task (map frame, m)"),
            BT::OutputPort<double>("dropoff_y", "Dropoff pose y of the next task (map frame, m)"),
            BT::OutputPort<double>("dropoff_yaw", "Dropoff pose yaw of the next task (rad)")};
    }

    BT::NodeStatus GetNextTask::tick()
    {
        auto task = task_queue->get_next_task();
        if(!task.has_value())
        {
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(logger(), "[%s] Starting task #%u: pickup (%.2f, %.2f, %.2f) -> dropoff (%.2f, %.2f, %.2f)",
                    name().c_str(), task->task_id,
                    task->pickup_pose.x, task->pickup_pose.y, task->pickup_pose.yaw,
                    task->dropoff_pose.x, task->dropoff_pose.y, task->dropoff_pose.yaw);

        setOutput("pickup_x", task->pickup_pose.x);
        setOutput("pickup_y", task->pickup_pose.y);
        setOutput("pickup_yaw", task->pickup_pose.yaw);
        setOutput("dropoff_x", task->dropoff_pose.x);
        setOutput("dropoff_y", task->dropoff_pose.y);
        setOutput("dropoff_yaw", task->dropoff_pose.yaw);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace amr_autonomy