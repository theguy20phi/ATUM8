#include "task.hpp"

namespace atum8
{
    void Task::start()
    {
        task = std::make_unique<pros::Task>(
            [this]()
            {
                this->taskFn();
            });
    }

    void Task::stop()
    {
        task = nullptr;
    }
}