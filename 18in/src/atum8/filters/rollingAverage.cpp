#include "rollingAverage.hpp"

namespace atum8
{
    RollingAverage::RollingAverage(int iSize) : size{iSize} {}

    void RollingAverage::update(double value)
    {
        values.insert(values.begin(), value);
        if (values.size() > size)
            values.pop_back();
        filteredVal = std::accumulate(values.begin(), values.end(), 0) / values.size();
    }

    void RollingAverage::reset() 
    {
        values.clear();
    }
}