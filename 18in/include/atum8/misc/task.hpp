/**
 * @file task.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides a simple parent class for other systems
 * to derive from in order to be treated like a task themselves and run a
 * set function, the taskFn, in parallel.
 * @version 0.2
 * @date 2023-02-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include "pros/rtos.hpp"
#include <memory>

namespace atum8
{
    using TaskFn = std::function<void()>;

    /**
     * @brief This provides a simple parent class for other systems
     * to derive from in order to be treated like a task themselves and run those
     * functions passed in in parallel.
     *
     */
    class Task
    {
    public:
        /**
         * @brief Constructs a new Task object.
         * 
         * @param iPriority 
         */
        Task(int iPriority = 8);

        /**
         * @brief Adds taskFns to be ran in parallel.
         * 
         * @param iTaskFns 
         */
        virtual void addTaskFns(const std::vector<TaskFn> iTaskFns);

        /**
         * @brief Starts the task (begins running taskFn).
         *
         */
        virtual void start();

        /**
         * @brief Stops the task (stops running taskFn).
         *
         */
        virtual void stop();

    protected:
        int priority;
        std::vector<TaskFn> taskFns;
        std::vector<std::shared_ptr<pros::Task>> tasks;
    };
} // namespace atum8
