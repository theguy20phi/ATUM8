/**
 * @file odometer.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This file provides the implementation for odometers.
 * Essentially wraps around an encoder (which may or may not be third-party),
 * and converts rotations to distance.
 * @version 0.1
 * @date 2023-03-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "pros/adi.hpp"
#include "okapi/api/units/QLength.hpp"
#include <memory>

namespace atum8
{
    /**
     * @brief This class provides the implementation for odometers.
     * Essentially wraps around an encoder (which may or may not be third-party),
     * and converts rotations to distance.
     *
     */
    class Odometer
    {
    public:
        /**
         * @brief The necessary odometer dimensions in order to perform odometry.
         *
         */
        struct Dimensions
        {
            okapi::QLength wheelCircum;
            okapi::QLength distanceToCenter;
        };

        /**
         * @brief Constructs a new Odometer object.
         *
         * @param portA
         * @param portB
         * @param multiplier
         * @param iDimensions
         */
        Odometer(char portA, char portB, double iMultiplier, const Dimensions &iDimensions);

        /**
         * @brief Gets the distance traveled since last called.
         *
         * @return okapi::QLength
         */
        okapi::QLength getDistance();

        /**
         * @brief Gets the odometers distance from the center of the bot.
         *
         * @return okapi::QLength
         */
        okapi::QLength getDistanceToCenter() const;

        /**
         * @brief Gets the raw tick count of the encoder.
         *
         * @return int
         */
        int getRawTicks() const;

    private:
        pros::ADIEncoder encoder;
        double multiplier;
        Dimensions dimensions;
        int prevTicks{0};
    };

    using UPOdometer = std::unique_ptr<Odometer>;
    using SPOdometer = std::shared_ptr<Odometer>;
}