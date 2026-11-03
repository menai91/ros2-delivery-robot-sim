#include "amr_autonomy/bt_drop_off_item.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("DropoffItem"); }
}

namespace amr_autonomy
{
    DropoffItem::DropoffItem(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload)
        : BT::StatefulActionNode(name, config), payload(std::move(in_payload))
    {
        if (payload == nullptr)
        {
            throw BT::RuntimeError("Payload is not valid");
        }
    }

    BT::PortsList DropoffItem::providedPorts()
    {
        return {};
    }

    BT::NodeStatus DropoffItem::onStart()
    {
        if (!payload->is_loaded())
        {
            RCLCPP_ERROR(logger(), "[%s] Cannot drop off, the robot is not carrying anything", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        //for now, stay in the DropoffItem node for 5 seconds
        end_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        RCLCPP_INFO(logger(), "[%s] Started", name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus DropoffItem::onRunning()
    {
        if(std::chrono::steady_clock::now() >= end_time)
        {
            payload->unload();
            RCLCPP_INFO(logger(), "[%s] Finished, item unloaded", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void DropoffItem::onHalted()
    {
    }

} // namespace amr_autonomy