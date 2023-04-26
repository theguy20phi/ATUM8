#pragma once

#include "filter.hpp"
#include <vector>
#include <numeric>

namespace atum8
{
    class RollingAverage : public Filter
    {
    public:
        RollingAverage(int iSize);

        void update(double value);

        void reset();

    private:
        std::vector<double> values;
        const int size;
    };

    using UPRollingAverage = std::unique_ptr<RollingAverage>;
    using SPRollingAverage = std::shared_ptr<RollingAverage>;
}