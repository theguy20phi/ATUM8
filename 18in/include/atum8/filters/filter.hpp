#pragma once

#include <memory>

namespace atum8
{
    class Filter
    {
    public:
        virtual double get();

        virtual double get(double value);

        virtual void update(double value);

        virtual void reset();

    protected:
        double filteredVal{0.0};
    };

    using UPFilter = std::unique_ptr<Filter>;
    using SPFilter = std::shared_ptr<Filter>;
}