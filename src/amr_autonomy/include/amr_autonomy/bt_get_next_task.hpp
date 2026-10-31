#pragma once

#include "behaviortree_cpp/action_node.h"
#include "amr_autonomy/task_queue.hpp"

namespace amr_autonomy
{
    class GetNextTask : public BT::SyncActionNode
    {
    public:
        explicit GetNextTask(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::TaskQueue> in_task_queue);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;


    private:
        std::shared_ptr<amr_autonomy::TaskQueue> task_queue;
    };
}