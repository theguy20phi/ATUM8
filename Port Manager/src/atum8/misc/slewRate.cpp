#include "slewRate.hpp"

namespace atum8
{
    SlewRate::SlewRate(double maxChange) : maxNegChange{-maxChange},
                                           maxPosChange{maxChange}
    {
    }

    SlewRate::SlewRate(double iMaxNegChange, double iMaxPosChange) : maxNegChange{iMaxNegChange},
                                                                     maxPosChange{iMaxPosChange}
    {
    }

    double SlewRate::slew(double reference)
    {
        const double difference{reference - output};
        output += std::clamp(difference, maxNegChange, maxPosChange);
        return output;
    }

    double SlewRate::getOutput() const
    {
        return output;
    }

    void SlewRate::reset()
    {
        output = 0;
    }
};