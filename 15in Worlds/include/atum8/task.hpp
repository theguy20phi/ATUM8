/**
 * @file task.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides a simple parent class for other systems
 * to derive from in order to be treated like a task themselves and run a
 * set function, the taskFn, in parallel.
 * @version 0.1
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
    /**
     * @brief This provides a simple parent class for other systems
     * to derive from in order to be treated like a task themselves and run a
     * set function, the taskFn, in parallel.
     *
     */
    class Task
    {
    public:
        /**
         * @brief This function will be ran in a pros task.
         *
         */
        virtual void taskFn() = 0;

        /**
         * @brief Starts the task (begins running taskFn).
         *
         */
        virtual void start();

        /**
         * @brief Stops the tas (stops running taskFn).
         *
         */
        virtual void stop();

    protected:
        int priority;
        std::unique_ptr<pros::Task> task{nullptr};
    };
} // namespace atum8
