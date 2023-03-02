#include "rollingAverage.hpp"

namespace atum8
{
    RollingAverage::RollingAverage(int iSize) : size{iSize} {}

    double RollingAverage::getAverage(double value)
    {
        update(value);
        return getAverage();
    }

    double RollingAverage::getAverage() const
    {
        return std::accumulate(values.begin(), values.end(), 0) / values.size();
    }

    void RollingAverage::update(double value)
    {
        values.insert(values.begin(), value);
        if (values.size() > size)
            values.pop_back();
    }
}