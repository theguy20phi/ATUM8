#pragma once

#include "poseEstimator.hpp"
#include "atum8/filters/filter.hpp"
#include "atum8/misc/constants.hpp"

namespace atum8
{
    class GPS
    {
    public:
        GPS(UPRawGPS iGps,
            SPFilter iXFilter,
            SPFilter iYFilter,
            SPFilter iHFilter,
            double iTrust = 1.0,
            const okapi::QTime &iTimeout = 1_s);

        Position getPosition(const Position &otherPosition);

        Position getPosition();

    private:
        UPRawGPS gps;
        SPFilter xFilter;
        SPFilter yFilter;
        SPFilter hFilter;
        const double trust;
        const okapi::QTime timeout;
    };

    using UPGPS = std::unique_ptr<GPS>;
    using SPGPS = std::shared_ptr<GPS>;

    class SPGPSBuilder
    {
    public:
        SPGPS build();

        SPGPSBuilder withPort(int8_t iPort);

        SPGPSBuilder withOffset(const Position &offset);

        SPGPSBuilder withXFilter(SPFilter iXFilter);

        SPGPSBuilder withYFilter(SPFilter iYFilter);

        SPGPSBuilder withHFilter(SPFilter iHFilter);

        SPGPSBuilder withTrust(double iTrust);

        SPGPSBuilder withTimeout(const okapi::QTime &iTimeout);

    private:
        int8_t port;
        double xOffset;
        double yOffset;
        SPFilter xFilter{std::make_shared<Filter>()};
        SPFilter yFilter{std::make_shared<Filter>()};
        SPFilter hFilter{std::make_shared<Filter>()};
        double trust{1.0};
        okapi::QTime timeout{1_s};
    };
}