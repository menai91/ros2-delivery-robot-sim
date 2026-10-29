#include "amr_autonomy/bt_move_to_position.hpp"

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("MoveToPosition"); }
}

namespace amr_autonomy
{
    MoveToPosition::MoveToPosition(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<amr_navigation::Navigator> nav)
        : BT::StatefulActionNode(name, config), navigator(std::move(nav))
    {
        if (navigator == nullptr)
        {
            throw BT::RuntimeError("Navigator is not valid");
        }
    }

    BT::PortsList MoveToPosition::providedPorts()
    {
        return {
            BT::InputPort<float>("x", "x position to move to"),
            BT::InputPort<float>("y", "y position to move to"),
            BT::InputPort<float>("yaw", 0.0, "yaw to rotate to when reaching the target x,y position"),

            // TODO: depending on how we use the output
            // BT::OutputPort<int>("error_code", "Nav2 / Navigator error code of the last goal")
        };
    }

    BT::NodeStatus MoveToPosition::onStart()
    {
        const auto x_position = getInput<float>("x");
        const auto y_position = getInput<float>("y");
        const auto yaw = getInput<float>("yaw");

        if (!x_position)
        {
            RCLCPP_ERROR(logger(), "[%s] Invalid input port 'x': %s", name().c_str(), x_position.error().c_str());
        }

        if (!y_position)
        {
            RCLCPP_ERROR(logger(), "[%s] Invalid input port 'y': %s", name().c_str(), y_position.error().c_str());
        }

        if (!yaw)
        {
            RCLCPP_ERROR(logger(), "[%s] Invalid input port 'yaw': %s", name().c_str(), yaw.error().c_str());
        }

        if (!x_position || !y_position || !yaw)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (!navigator->start(x_position.value(), y_position.value(), yaw.value()))
        {
            RCLCPP_ERROR(logger(), "[%s] Navigator failed to start goal (%.2f, %.2f, %.2f), error code: %u",
                         name().c_str(), x_position.value(), y_position.value(), yaw.value(), navigator->get_error_code());

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveToPosition::onRunning()
    {
        switch (navigator->get_status())
        {
        case amr_navigation::NavStatus::Running:
        {
            return BT::NodeStatus::RUNNING;
        }
        break;
        case amr_navigation::NavStatus::Succeeded:
        {
            return BT::NodeStatus::SUCCESS;
        }
        break;
        case amr_navigation::NavStatus::Failed:
        {
            return BT::NodeStatus::FAILURE;
        }
        break;
        case amr_navigation::NavStatus::Cancelled:
        {
            RCLCPP_WARN(logger(), "[%s] Navigation was cancelled while the node was running", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        break;
        default:
            // shouldnt be coming in here
            RCLCPP_ERROR(logger(), "[%s] Unexpected navigator status %d", name().c_str(), static_cast<int>(navigator->get_status()));
            return BT::NodeStatus::FAILURE;
            break;
        }
    }

    void MoveToPosition::onHalted()
    {
        RCLCPP_INFO(logger(), "[%s] Halted, cancelling navigation", name().c_str());
        navigator->cancel();
    }

} // namespace amr_autonomy