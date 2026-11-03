#include "amr_autonomy/bt_clear_payload.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("ClearPayload"); }
}

namespace amr_autonomy
{
    ClearPayload::ClearPayload(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload)
        : BT::SyncActionNode(name, config), payload(std::move(in_payload))
    {
        if (payload == nullptr)
        {
            throw BT::RuntimeError("Payload is not valid");
        }
    }

    BT::PortsList ClearPayload::providedPorts()
    {
        return {};
    }

    BT::NodeStatus ClearPayload::tick()
    {
        if (payload->is_loaded())
        {
            RCLCPP_WARN(logger(), "[%s] Payload cleared without a station, assuming the bin was removed by an operator", name().c_str());
        }

        payload->unload();
        return BT::NodeStatus::SUCCESS;
    }
} // namespace amr_autonomy
