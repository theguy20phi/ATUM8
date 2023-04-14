#include "constants.hpp"

namespace atum8
{
    void waitFor(const std::function<bool()> &condition, const okapi::QTime &maxTime)
    {
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        // While condition isn't met and (haven't used max time OR max time wasn't set).
        while (!condition() && ((pros::millis() * okapi::millisecond - startTime < maxTime) || (maxTime == 0_s)))
            pros::delay(stdDelay);
    }
}