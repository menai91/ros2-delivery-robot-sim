#include "amr_autonomy/delivery_request_server.hpp"

#include <stdexcept>

namespace amr_autonomy
{
    DeliveryRequestServer::DeliveryRequestServer(const rclcpp::Node::SharedPtr &node, std::shared_ptr<amr_autonomy::TaskQueue> in_task_queue, amr_navigation::StationMap in_stations)
        : task_queue(std::move(in_task_queue)), stations(std::move(in_stations)), logger(node->get_logger())
    {
        if (!task_queue)
        {
            throw std::runtime_error("Invalid task_queue passed in");
        }

        service = node->create_service<DeliverRequestSrv>("delivery_request", [this](const std::shared_ptr<DeliverRequestSrv::Request> request
            , std::shared_ptr<DeliverRequestSrv::Response> response)
            {
                handle_request(request, response);
            });

        RCLCPP_INFO(logger, "Accepting delivery requests on '%s' (%zu stations loaded)", service->get_service_name(), stations.size());
    }

    void DeliveryRequestServer::handle_request(const std::shared_ptr<DeliverRequestSrv::Request> request, std::shared_ptr<DeliverRequestSrv::Response> response)
    {
        amr_autonomy::DeliveryTask new_task;

        auto parse_location = [this](const amr_interfaces::msg::Location &location, amr_navigation::Pose2D &out_position) -> bool
        {
            if(!location.station.empty())
            {
                const auto entry = stations.find(location.station);
                if(entry != stations.end())
                {
                    out_position = entry->second;
                    return true;
                }

                return false;
            }

            out_position = {location.x, location.y, location.yaw};
            return true;
        };

        if (!parse_location(request->pickup, new_task.pickup_position))
        {
            response->accepted = false;
            response->message = "unknown pickup station '" + request->pickup.station + "'";
            RCLCPP_WARN(logger, "Rejected delivery request: %s", response->message.c_str());
            return;
        }

        if (!parse_location(request->dropoff, new_task.dropoff_position))
        {
            response->accepted = false;
            response->message = "unknown dropoff station '" + request->dropoff.station + "'";
            RCLCPP_WARN(logger, "Rejected delivery request: %s", response->message.c_str());
            return;
        }

        task_queue->push(new_task);
        response->accepted = true;
        response->message = "task has been queued";

        const auto &pickup = new_task.pickup_position;
        const auto &dropoff = new_task.dropoff_position;
        RCLCPP_INFO(logger, "Queued delivery: pickup (%.2f, %.2f, %.2f) -> dropoff (%.2f, %.2f, %.2f), %zu task(s) waiting",
                    pickup.x, pickup.y, pickup.yaw, dropoff.x, dropoff.y, dropoff.yaw, task_queue->size());
    }
} // namespace amr_autonomy