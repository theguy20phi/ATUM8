/**
 * @file settledChecker.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Implements a simple algorithm to check if a system
 * has settled.
 * @version 0.3
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "pros/rtos.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"

namespace atum8
{
    /**
     * @brief Implements a simple algorithm to check if a system
     * has settled. Must be given the type of primary unit and the unit of
     * its derivative with respect to time (i.e. distance and velocity or
     * angle and angular speed).
     *
     * @tparam Unit
     * @tparam UnitDeriv
     */
    template <typename Unit = double, typename UnitDeriv = double>
    class SettledChecker
    {
    public:
        /**
         * @brief Constructs a new SettledChecker object.
         *
         * @param iMaxDistance Boundary for primary unit.
         * @param iMaxVelocity Boundary for secondary unit.
         * @param iMinTime How long the current value has to remain in boundaries
         * to be considered settled.
         */
        SettledChecker(const Unit &iMaxDistance,
                       const UnitDeriv &iMaxVelocity,
                       const okapi::QTime &iMinTime) : maxDistance{iMaxDistance},
                                                       maxVelocity{iMaxVelocity},
                                                       minTime{iMinTime}
        {
        }

        /**
         * @brief Passes on the difference between reference and state to isSettled(const Unit&).
         *
         * @param state
         * @param reference
         * @return true
         * @return false
         */
        bool isSettled(const Unit &state, const Unit &reference)
        {
            return isSettled(reference - state);
        }

        /**
         * @brief Given the current value of the system, will check if a system has
         * been within a boundary for both the value and its derivative with respect to time
         * and has been there for a certain amount of time.
         *
         * @param distance
         * @return true
         * @return false
         */
        bool isSettled(const Unit &distance)
        {
            const okapi::QTime time{pros::millis() * okapi::millisecond};
            const UnitDeriv velocity{(distance - prevDistance) / (time - prevTime)};
            prevDistance = distance;
            prevTime = time;
            if (abs(distance) <= maxDistance && (abs(velocity) <= maxVelocity || !maxVelocity.getValue()))
            {
                settled = (time - prevTimeInBounds >= minTime) || !minTime.getValue();
                return settled;
            }
            prevTimeInBounds = time;
            settled = false;
            return settled;
        }

        /**
         * @brief Returns if the system was settled based on the last call of
         * isSettled(const Unit&).
         *
         * @return true
         * @return false
         */
        bool isSettled()
        {
            return settled;
        }

    private:
        bool settled{false};
        Unit maxDistance;
        UnitDeriv maxVelocity;
        okapi::QTime minTime;
        Unit prevDistance;
        okapi::QTime prevTime;
        okapi::QTime prevTimeInBounds;
    };

    template <typename Unit = double, typename UnitDeriv = double>
    using UPSettledChecker = std::unique_ptr<SettledChecker<Unit, UnitDeriv>>;

    template <typename Unit = double, typename UnitDeriv = double>
    using SPSettledChecker = std::shared_ptr<SettledChecker<Unit, UnitDeriv>>;

    using UPLateralSettledChecker = UPSettledChecker<okapi::QLength, okapi::QSpeed>;
    using SPLateralSettledChecker = SPSettledChecker<okapi::QLength, okapi::QSpeed>;

    using UPAngularSettledChecker = UPSettledChecker<okapi::QAngle, okapi::QAngularSpeed>;
    using SPAngularSettledChecker = SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed>;
}