#include "amr_autonomy/task_queue.hpp"

#include <utility>

namespace amr_autonomy
{
    void TaskQueue::push(amr_autonomy::DeliveryTask task)
    {
        task.task_id = ++next_task_id;
        task_queue.push_back(task);
    }

    std::optional<amr_autonomy::DeliveryTask> TaskQueue::pop()
    {
        if(empty())
        {
            return {};
        }
        
        amr_autonomy::DeliveryTask task = task_queue.front();
        task_queue.pop_front();

        return task;
    }

    std::size_t TaskQueue::size() const
    {
        return task_queue.size();
    }

    bool TaskQueue::empty() const
    {
        return task_queue.empty();
    }
}