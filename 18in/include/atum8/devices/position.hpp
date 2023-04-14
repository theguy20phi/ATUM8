#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/odometry/odomMath.hpp"

using namespace okapi::literals;

namespace atum8
{
    struct Position
    {
        okapi::QLength x;
        okapi::QLength y;
        okapi::QAngle h;
    };

    okapi::QLength distance(const Position &a, const Position &b);

    okapi::QAngle angle(const Position &state, const Position &point);
}