/**
 * @file filter.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Implements a simple interface for other filters to obey, as
 * well as a simple pass-through filter that can function as a null object.
 * @version 0.1
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <memory>

namespace atum8
{
    /**
     * @brief Implements a simple interface for other filters to obey, as
     * well as a simple pass-through filter that can function as a null object.
     *
     */
    class Filter
    {
    public:
        /**
         * @brief Gets the filtered value based on previous input.
         *
         * @return double
         */
        virtual double get();

        /**
         * @brief Gets the filtered value based on input and previous input.
         *
         * @param value
         * @return double
         */
        virtual double get(double value);

        /**
         * @brief Updates the filtered value without returning.
         *
         * @param value
         */
        virtual void update(double value);

    protected:
        double filteredVal{0.0};
    };

    using UPFilter = std::unique_ptr<Filter>;
    using SPFilter = std::shared_ptr<Filter>;
}