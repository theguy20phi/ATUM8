#include "controller.hpp"

namespace atum8
{
    double Controller::getOutput() const
    {
        return output;
    }

    bool Controller::sampleTimePassed()
    {
        const long unsigned int time{pros::millis()};
        if (time - prevTime >= stdDelay)
        {
            prevTime = time;
            return true;
        }
        return false;
    }

    void Controller::reset()
    {
        output = 0;
    }
}