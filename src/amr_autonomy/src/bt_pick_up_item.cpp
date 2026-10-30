#include "amr_autonomy/bt_pick_up_item.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("PickupItem"); }
}

namespace amr_autonomy
{
    PickupItem::PickupItem(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    BT::PortsList PickupItem::providedPorts()
    {
        return {};
    }

    BT::NodeStatus PickupItem::onStart()
    {
        //for now, stay in the DropoffItem node for 5 seconds
        end_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        RCLCPP_INFO(logger(), "[%s] Started", name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PickupItem::onRunning()
    {
        if(std::chrono::steady_clock::now() >= end_time)
        {
            RCLCPP_INFO(logger(), "[%s] Finished", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void PickupItem::onHalted()
    {
    }

} // namespace amr_autonomy