#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "atum8/misc/constants.hpp"

using namespace okapi::literals;

namespace atum8
{
    struct Position
    {
        okapi::QLength x{0_in};
        okapi::QLength y{0_in};
        okapi::QAngle h{0_deg};
    };

    okapi::QLength distance(const Position &a, const Position &b);

    okapi::QAngle angle(const Position &state, const Position &point);

    Position accountForSide(const Position &position, const Color &color, bool reverseH = false);
}