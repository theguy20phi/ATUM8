#pragma once

#include "pros/adi.hpp"
#include "atum8/filters/rollingMedian.hpp"

namespace atum8
{
    class Potentiometer
    {
    public:
        Potentiometer(uint8_t smartPort,
                      uint8_t adiPort,
                      const std::vector<double> &iPositionMap,
                      int medFilterSize = 10);

        Potentiometer(uint8_t port,
                      const std::vector<double> &iPositionMap,
                      int medFilterSize = 10);

        double getPosition();

        int getMappedPosition();

    private:
        pros::ADIPotentiometer potentiometer;
        std::vector<double> positionMap;
        RollingMedian filter;
    };

    using UPPotentiometer = std::unique_ptr<Potentiometer>;
    using SPPotentiometer = std::shared_ptr<Potentiometer>;
}