#pragma once

#include "filter.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace atum8
{
    class RollingMedian : public Filter
    {
    public:
        RollingMedian(int iSize);

        void update(double value);

        void reset();

    private:
        std::vector<double> values;
        const int size;
        const int lower;
        const int upper;
    };

    using UPRollingMedian = std::unique_ptr<RollingMedian>;
    using SPRollingMedian = std::shared_ptr<RollingMedian>;
}