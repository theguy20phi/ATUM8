#pragma once

#include "pros/rtos.hpp"
#include <memory>

namespace atum8
{
    using TaskFn = std::function<void()>;

    class Task
    {
    public:
        Task(int iPriority = 8);

        virtual void addTaskFns(const std::vector<TaskFn> iTaskFns);

        virtual void start();

        virtual void stop();

    protected:
        int priority;
        std::vector<TaskFn> taskFns;
        std::vector<std::shared_ptr<pros::Task>> tasks;
    };
} // namespace atum8
