#pragma once

#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "amr_autonomy/payload.hpp"

namespace amr_autonomy
{
    // Marks the robot as empty without unloading at a station, e.g. when an operator has removed the bin
    class ClearPayload : public BT::SyncActionNode
    {
    public:
        explicit ClearPayload(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<amr_autonomy::Payload> payload;
    };
}
