#pragma once

#include <memory>

#include "amr_autonomy/task_queue.hpp"
#include "amr_interfaces/srv/delivery_request.hpp"
#include "amr_navigation/station_loader.hpp"

#include "rclcpp/node.hpp"

namespace amr_autonomy
{
    class DeliveryRequestServer
    {
    public:
        DeliveryRequestServer(const rclcpp::Node::SharedPtr &node, std::shared_ptr<amr_autonomy::TaskQueue> task_queue, amr_navigation::StationMap stations);

        //delete the copy constructors
        DeliveryRequestServer(const DeliveryRequestServer &) = delete;
        DeliveryRequestServer &operator=(const DeliveryRequestServer &) = delete;

    private:
        using DeliverRequestSrv = amr_interfaces::srv::DeliveryRequest;
        
        void handle_request(const std::shared_ptr<DeliverRequestSrv::Request> request, std::shared_ptr<DeliverRequestSrv::Response> response);
    
    private:
        std::shared_ptr<amr_autonomy::TaskQueue> task_queue;
        amr_navigation::StationMap stations;
        rclcpp::Logger logger;
        rclcpp::Service<DeliverRequestSrv>::SharedPtr service;
    };
}