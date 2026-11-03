#pragma once

#include <memory>

#include "behaviortree_cpp/condition_node.h"
#include "amr_autonomy/payload.hpp"

namespace amr_autonomy
{
    // SUCCESS if the robot is carrying something, FAILURE if not
    class IsItemOnBoard : public BT::ConditionNode
    {
    public:
        explicit IsItemOnBoard(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<amr_autonomy::Payload> payload;
    };
}
