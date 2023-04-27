#pragma once

#include "pros/rtos.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"

namespace atum8
{
    template <typename Unit = double, typename UnitDeriv = double>
    class SettledChecker
    {
    public:
        SettledChecker(const Unit &iMaxDistance,
                       const UnitDeriv &iMaxVelocity,
                       const okapi::QTime &iMinTime) : maxDistance{iMaxDistance},
                                                       maxVelocity{iMaxVelocity},
                                                       minTime{iMinTime}
        {
        }

        bool isSettled(const Unit &state, const Unit &reference)
        {
            return isSettled(reference - state);
        }

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