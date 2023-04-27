#pragma once

#include "pros/adi.hpp"
#include "okapi/api/units/QLength.hpp"
#include <memory>

namespace atum8
{
    class Odometer
    {
    public:
        struct Dimensions
        {
            okapi::QLength wheelCircum;
            okapi::QLength distanceToCenter;
        };

        Odometer(char portA, char portB, double iMultiplier, const Dimensions &iDimensions, bool iReversed);

        okapi::QLength getDistance();

        okapi::QLength getDistanceToCenter() const;

        int getRawTicks() const;

    private:
        pros::ADIEncoder encoder;
        double multiplier;
        Dimensions dimensions;
        bool reversed;
        int prevTicks{0};
    };

    using UPOdometer = std::unique_ptr<Odometer>;
    using SPOdometer = std::shared_ptr<Odometer>;
}