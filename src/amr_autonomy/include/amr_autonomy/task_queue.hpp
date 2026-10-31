#pragma once

#include <deque>
#include <optional>

#include "amr_autonomy/delivery_task.hpp"

namespace amr_autonomy
{
    class TaskQueue
    {
    public:
        void push(amr_autonomy::DeliveryTask task);
        std::optional<amr_autonomy::DeliveryTask> pop();
        std::size_t size() const;
        bool empty() const;

    private:
        std::deque<amr_autonomy::DeliveryTask> task_queue;
        unsigned int next_task_id = 0;
    };
}