/**
 * @file rollingAverage.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides an implementation for a rolling average.
 * @version 0.1
 * @date 2023-03-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <vector>
#include <numeric>
#include <memory>

namespace atum8
{
    /**
     * @brief Provides an implementation of a rolling average. A rolling
     * average simply means that the average of a continually updating value is
     * calculated only as the average of the last few values (which, here, is
     * determined by the template parameter).
     *
     * @tparam size
     */
    class RollingAverage
    {
    public:
        /**
         * @brief Constructs a new RollingAverage object.
         *
         * @param iSize
         */
        RollingAverage(int iSize);

        /**
         * @brief Gets the rolling average given a new value.
         *
         * @param value
         * @return double
         */
        double getAverage(double value);

        /**
         * @brief Gets the current rolling average.
         *
         * @return double
         */
        double getAverage() const;

        /**
         * @brief Updates the values.
         *
         * @param value
         */
        void update(double value);

    private:
        std::vector<double> values;
        const double size;
    };

    using UPRollingAverage = std::unique_ptr<RollingAverage>;
    using SPRollingAverage = std::shared_ptr<RollingAverage>;
}