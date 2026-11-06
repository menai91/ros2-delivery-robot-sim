#pragma once

#include <deque>
#include <optional>

#include "amr_autonomy/delivery_task.hpp"

namespace amr_autonomy
{
    enum TaskOutcome : unsigned int
    {
        Task_Completed,
        Task_Failed_Discard,
        Task_Failed_Requeue,
    };

    class TaskQueue
    {
    public:
        void push(amr_autonomy::DeliveryTask task);
        std::optional<amr_autonomy::DeliveryTask> get_next_task();

        bool set_task_outcome(TaskOutcome outcome);

        std::size_t size() const;
        bool empty() const;

    private:
        std::deque<amr_autonomy::DeliveryTask> task_queue;
        std::optional<amr_autonomy::DeliveryTask> current_task;

        unsigned int next_task_id = 0;
    };
}