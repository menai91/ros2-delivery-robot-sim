#include "amr_autonomy/bt_pick_up_item.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("PickupItem"); }
}

namespace amr_autonomy
{
    PickupItem::PickupItem(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload)
        : BT::StatefulActionNode(name, config), payload(std::move(in_payload))
    {
        if (payload == nullptr)
        {
            throw BT::RuntimeError("Payload is not valid");
        }
    }

    BT::PortsList PickupItem::providedPorts()
    {
        return {};
    }

    BT::NodeStatus PickupItem::onStart()
    {
        // the robot carries one bin at most
        if (payload->is_loaded())
        {
            RCLCPP_ERROR(logger(), "[%s] Cannot pick up, the robot is already carrying something", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        //for now, stay in the PickupItem node for 5 seconds
        end_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        RCLCPP_INFO(logger(), "[%s] Started", name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PickupItem::onRunning()
    {
        if(std::chrono::steady_clock::now() >= end_time)
        {
            payload->load();
            RCLCPP_INFO(logger(), "[%s] Finished, item loaded", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void PickupItem::onHalted()
    {
    }

} // namespace amr_autonomy