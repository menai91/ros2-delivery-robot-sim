#pragma once

#include <memory>
#include <string>

#include "amr_autonomy/task_queue.hpp"
#include "behaviortree_cpp/action_node.h"

namespace amr_autonomy
{
    class SetTaskOutcome : public BT::SyncActionNode
    {
    public:
        explicit SetTaskOutcome(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::TaskQueue> in_task_queue);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
        
    private:
        std::shared_ptr<amr_autonomy::TaskQueue> task_queue;
    };
}