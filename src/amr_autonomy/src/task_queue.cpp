#include "amr_autonomy/task_queue.hpp"

#include <utility>

#include "rclcpp/logging.hpp"

namespace
{
    rclcpp::Logger logger() { return rclcpp::get_logger("TaskQueue"); }
}

namespace amr_autonomy
{
    void TaskQueue::push(amr_autonomy::DeliveryTask task)
    {
        task.task_id = ++next_task_id;
        task_queue.push_back(task);
    }

    std::optional<amr_autonomy::DeliveryTask> TaskQueue::get_next_task()
    {
        if (current_task.has_value())
        {
            RCLCPP_INFO(logger(), "Resuming task #%u", current_task->task_id);
            return current_task;
        }
        else if (empty())
        {
            return {};
        }

        current_task = task_queue.front();
        task_queue.pop_front();

        return current_task;
    }

    bool TaskQueue::set_task_outcome(TaskOutcome outcome)
    {
        if (!current_task.has_value())
        {
            // no existing running task, so do not set outcome
            RCLCPP_ERROR(logger(), "Cannot set a task outcome: there is no current task");
            return false;
        }

        const unsigned int task_id = current_task->task_id;

        // no default case, so the compiler warns (-Wswitch) if a new TaskOutcome is not handled here
        switch (outcome)
        {
        case TaskOutcome::Task_Completed:
        {
            RCLCPP_INFO(logger(), "Task #%u completed", task_id);
        }
        break;
        case TaskOutcome::Task_Failed_Discard:
        {
            RCLCPP_ERROR(logger(), "Task #%u failed and was discarded", task_id);
        }
        break;
        case TaskOutcome::Task_Failed_Requeue:
        {
            task_queue.emplace_back(std::move(current_task.value()));
            RCLCPP_WARN(logger(), "Task #%u failed and was re-queued at the back (%zu waiting)", task_id, task_queue.size());
        }
        break;
        }

        current_task.reset();
        return true;
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