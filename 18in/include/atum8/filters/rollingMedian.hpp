/**
 * @file rollingMedian.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Implements a rolling median filter.
 * @version 0.1
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "filter.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace atum8
{
    /**
     * @brief Implements a rolling median filter.
     *
     */
    class RollingMedian : public Filter
    {
    public:
        /**
         * @brief Constructs a new RollingMedian object.
         *
         * @param iSize
         */
        RollingMedian(int iSize);

        /**
         * @brief Updates the values.
         *
         * @param value
         */
        void update(double value);

    private:
        std::vector<double> values;
        const int size;
        const int lower;
        const int upper;
    };

    using UPRollingMedian = std::unique_ptr<RollingMedian>;
    using SPRollingMedian = std::shared_ptr<RollingMedian>;
}