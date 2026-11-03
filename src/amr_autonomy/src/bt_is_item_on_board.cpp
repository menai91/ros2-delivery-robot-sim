#include "amr_autonomy/bt_is_item_on_board.hpp"

namespace amr_autonomy
{
    IsItemOnBoard::IsItemOnBoard(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_autonomy::Payload> in_payload)
        : BT::ConditionNode(name, config), payload(std::move(in_payload))
    {
        if (payload == nullptr)
        {
            throw BT::RuntimeError("Payload is not valid");
        }
    }

    BT::PortsList IsItemOnBoard::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsItemOnBoard::tick()
    {
        return payload->is_loaded() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
} // namespace amr_autonomy
