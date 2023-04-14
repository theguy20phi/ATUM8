#include "task.hpp"

namespace atum8
{
    Task::Task(int iPriority) : priority{iPriority}
    {
    }

    void Task::addTaskFns(const std::vector<TaskFn> iTaskFns) {
        taskFns = iTaskFns;
    }

    void Task::start()
    {
        for (TaskFn taskFn : taskFns)
            tasks.push_back(std::make_shared<pros::Task>(taskFn, priority));
    }

    void Task::stop()
    {
        for (auto task : tasks)
            task = nullptr;
    }
}