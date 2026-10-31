#include "amr_autonomy/bt_get_next_task.hpp"

#include <string>

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
        auto task = task_queue->pop();
        if(!task.has_value())
        {
            return BT::NodeStatus::FAILURE;
        }

        setOutput("pickup_x", task->pickup_position.x);
        setOutput("pickup_y", task->pickup_position.y);
        setOutput("pickup_yaw", task->pickup_position.yaw);
        setOutput("dropoff_x", task->dropoff_position.x);
        setOutput("dropoff_y", task->dropoff_position.y);
        setOutput("dropoff_yaw", task->dropoff_position.yaw);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace amr_autonomy