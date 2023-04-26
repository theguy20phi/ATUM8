#pragma once

#include "atum8/misc/constants.hpp"
#include "okapi/api/units/QAngle.hpp"

namespace atum8
{
    class Imus
    {
    public:
        Imus(const std::vector<int> &ports);

        double get_rotation();

        okapi::QAngle get_delta();

        void reset();

        void tare_rotation();

    private:
        okapi::QAngle prevHeading{0_deg};
        std::vector<pros::Imu> imus;
    };

    using UPImus = std::unique_ptr<Imus>;
    using SPImus = std::shared_ptr<Imus>;
}