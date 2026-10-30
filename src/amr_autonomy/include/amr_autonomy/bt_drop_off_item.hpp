#pragma once

#include <chrono>

#include "behaviortree_cpp/action_node.h"

namespace amr_autonomy
{
    class DropoffItem : public BT::StatefulActionNode
    {
    public:
        explicit DropoffItem(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        virtual BT::NodeStatus onStart() override;
        virtual BT::NodeStatus onRunning() override;
        virtual void onHalted() override;

    private:
        std::chrono::steady_clock::time_point end_time;
    };
}