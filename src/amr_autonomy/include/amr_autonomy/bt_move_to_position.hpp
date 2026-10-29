#pragma once

#include "behaviortree_cpp/action_node.h"
#include "amr_navigation/navigator.hpp"

namespace amr_autonomy
{
    class MoveToPosition : public BT::StatefulActionNode
    {
    public:
        explicit MoveToPosition(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_navigation::Navigator> nav);

        static BT::PortsList providedPorts();

        virtual BT::NodeStatus onStart() override;
        virtual BT::NodeStatus onRunning() override;
        virtual void onHalted() override;

    private:
        std::shared_ptr<amr_navigation::Navigator> navigator;
    };
} // namespace amr_autonomy