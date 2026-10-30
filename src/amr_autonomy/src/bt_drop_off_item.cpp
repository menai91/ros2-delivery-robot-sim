#include "amr_autonomy/bt_drop_off_item.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("DropoffItem"); }
}

namespace amr_autonomy
{
    DropoffItem::DropoffItem(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    BT::PortsList DropoffItem::providedPorts()
    {
        return {};
    }

    BT::NodeStatus DropoffItem::onStart()
    {
        //for now, stay in the DropoffItem node for 5 seconds
        end_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        RCLCPP_INFO(logger(), "[%s] Started", name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus DropoffItem::onRunning()
    {
        if(std::chrono::steady_clock::now() >= end_time)
        {
            RCLCPP_INFO(logger(), "[%s] Finished", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void DropoffItem::onHalted()
    {
    }

} // namespace amr_autonomy