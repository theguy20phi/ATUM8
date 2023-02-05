#include "controller.hpp"

namespace atum8
{
    double Controller::getOutput() const
    {
        return output;
    }

    bool Controller::sampleTimePassed()
    {
        const long unsigned int currentTime{pros::millis()};
        if (currentTime - prevTime >= stdDelay)
        {
            prevTime = currentTime;
            return true;
        }
        return false;
    }
}