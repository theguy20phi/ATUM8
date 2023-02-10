#include "slewRate.hpp"

namespace atum8
{
    SlewRate::SlewRate(double iMaxChange) : maxChange{iMaxChange}
    {
    }

    double SlewRate::slew(double reference)
    {
        const double difference{reference - output};
        output += std::clamp(difference, -maxChange, maxChange);
        return output;
    }

    double SlewRate::getOutput() const
    {
        return output;
    }

    void SlewRate::setMaxChange(double iMaxChange)
    {
        maxChange = iMaxChange;
    }

    double SlewRate::getMaxChange() const
    {
        return maxChange;
    }

    void SlewRate::reset()
    {
        output = 0;
    }
};