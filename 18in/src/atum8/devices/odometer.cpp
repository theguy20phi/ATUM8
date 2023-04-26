#include "odometer.hpp"

namespace atum8
{
    Odometer::Odometer(char portA,
                       char portB,
                       double iMultiplier,
                       const Dimensions &iDimensions,
                       bool iReversed) : encoder{portA, portB},
                                                        multiplier{iMultiplier},
                                                        dimensions{iDimensions},
                                                        reversed{iReversed}
    {
        encoder.reset();
    }

    okapi::QLength Odometer::getDistance()
    {
        const int ticks{encoder.get_value()};
        const int ticksDiff{ticks - prevTicks};
        prevTicks = ticks;
        const okapi::QLength distance{ticksDiff * multiplier * dimensions.wheelCircum};
        return reversed ? -1 * distance : distance;
    }

    okapi::QLength Odometer::getDistanceToCenter() const
    {
        return dimensions.distanceToCenter;
    }

    int Odometer::getRawTicks() const
    {
        return encoder.get_value();
    }
}