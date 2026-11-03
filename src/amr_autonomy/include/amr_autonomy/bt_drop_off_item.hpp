#pragma once

#include <chrono>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "amr_autonomy/payload.hpp"

namespace amr_autonomy
{
    class DropoffItem : public BT::StatefulActionNode
    {
    public:
        explicit DropoffItem(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload);

        static BT::PortsList providedPorts();

        virtual BT::NodeStatus onStart() override;
        virtual BT::NodeStatus onRunning() override;
        virtual void onHalted() override;

    private:
        std::shared_ptr<amr_autonomy::Payload> payload;
        std::chrono::steady_clock::time_point end_time;
    };
}