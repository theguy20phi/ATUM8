#pragma once

#include <memory>
#include <algorithm>

namespace atum8
{
    class SlewRate
    {
    public:
        SlewRate(double maxChange);

        SlewRate(double iMaxNegChange, double iMaxPosChange);

        double slew(double reference);

        double getOutput() const;

        void reset();

    private:
        double output{0};
        double maxPosChange{0};
        double maxNegChange{0};
    };

    using UPSlewRate = std::unique_ptr<SlewRate>;
    using SPSlewRate = std::shared_ptr<SlewRate>;
}