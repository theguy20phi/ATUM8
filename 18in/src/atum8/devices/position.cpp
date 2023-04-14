#include "position.hpp"

namespace atum8
{

    okapi::QLength distance(const Position &a, const Position &b)
    {
        const okapi::QLength dx{b.x - a.x};
        const okapi::QLength dy{b.y - a.y};
        return okapi::sqrt(dx * dx + dy * dy);
    }

    okapi::QAngle angle(const Position &state, const Position &point)
    {
        const okapi::QLength dx{point.x - state.x};
        const okapi::QLength dy{point.y - state.y};
        const okapi::QAngle dh{90_deg - okapi::atan2(dy, dx) - state.h};
        return okapi::OdomMath::constrainAngle180(dh);
    }

}