#include "filter.hpp"

namespace atum8
{
    double Filter::get()
    {
        return filteredVal;
    }

    double Filter::get(double value)
    {
        update(value);
        return filteredVal;
    }

    void Filter::update(double value)
    {
        filteredVal = value;
    }

    void Filter::reset()
    {
        filteredVal = 0;
    }
}