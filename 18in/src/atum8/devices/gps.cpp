#include "gps.hpp"

namespace atum8
{
    GPS::GPS(UPRawGPS iGps,
             SPFilter iXFilter,
             SPFilter iYFilter,
             SPFilter iHFilter,
             double iTrust,
             const okapi::QTime &iTimeout) : gps{std::move(iGps)},
                                             xFilter{iXFilter},
                                             yFilter{iYFilter},
                                             hFilter{iHFilter},
                                             trust{iTrust},
                                             timeout{iTimeout}
    {
    }

    Position GPS::getPosition(const Position &otherPosition)
    {
        Position gpsPosition{getPosition()};
        const okapi::QLength avgX{trust * gpsPosition.x + (1 - trust) * otherPosition.x};
        const okapi::QLength avgY{trust * gpsPosition.y + (1 - trust) * otherPosition.y};
        const okapi::QAngle avgH{trust * gpsPosition.h + (1 - trust) * otherPosition.h};
        return {avgX, avgY, avgH};
    }

    Position GPS::getPosition()
    {
        xFilter->reset();
        yFilter->reset();
        hFilter->reset();
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (pros::millis() * okapi::millisecond - startTime <= timeout)
        {
            const auto gpsData = gps->get_status();
            xFilter->update(gpsData.x);
            yFilter->update(gpsData.y);
            hFilter->update(gps->get_heading());
        }
        return {xFilter->get() * okapi::meter,
                yFilter->get() * okapi::meter,
                hFilter->get() * okapi::degree};
    }

    SPGPS SPGPSBuilder::build()
    {
        return std::make_shared<GPS>(std::make_unique<pros::GPS>(port, xOffset, yOffset),
                                     xFilter,
                                     yFilter,
                                     hFilter,
                                     trust,
                                     timeout);
    }

    SPGPSBuilder SPGPSBuilder::withPort(int8_t iPort)
    {
        port = iPort;
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withOffset(const Position &offset)
    {
        xOffset = offset.x.convert(okapi::meter);
        yOffset = offset.y.convert(okapi::meter);
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withXFilter(SPFilter iXFilter)
    {
        xFilter = iXFilter;
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withYFilter(SPFilter iYFilter)
    {
        yFilter = iYFilter;
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withHFilter(SPFilter iHFilter)
    {
        hFilter = iHFilter;
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withTrust(double iTrust)
    {
        trust = iTrust;
        return *this;
    }

    SPGPSBuilder SPGPSBuilder::withTimeout(const okapi::QTime &iTimeout)
    {
        timeout = iTimeout;
        return *this;
    }
}