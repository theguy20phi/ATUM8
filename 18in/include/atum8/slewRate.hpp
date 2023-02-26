/**
 * @file slewRate.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Implements a slew rate, which can be used to decrease the
 * max acceleration of a system.
 * @version 0.1
 * @date 2023-02-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <memory>
#include <algorithm>

namespace atum8
{
    /**
     * @brief Implements a slew rate, which can be used to decrease the
     * max acceleration of a system.
     *
     */
    class SlewRate
    {
    public:
        /**
         * @brief Constructs a new SlewRate object.
         * 
         * @param iMaxChange The max change allowed in the current value.
         */
        SlewRate(double iMaxChange);

        /**
         * @brief If the reference is within a max change of the
         * current output, returns the reference. Otherwise moves the
         * current output one maxChange in the direction of the reference.
         * 
         * @param reference The desired reference output.
         * @return double 
         */
        double slew(double reference);

        /**
         * @brief Gets the current output.
         * 
         * @return double 
         */
        double getOutput() const;

        /**
         * @brief Sets the maxChange.
         * 
         * @param iMaxChange The new max change allowed.
         */
        void setMaxChange(double iMaxChange);

        /**
         * @brief Gets the maxChange.
         * 
         * @return double 
         */
        double getMaxChange() const;
        
        /**
         * @brief Resets output to 0.
         * 
         */
        void reset();

    private:
        double output{0};
        double maxChange{0};
    };

    using UPSlewRate = std::unique_ptr<SlewRate>;
    using SPSlewRate = std::shared_ptr<SlewRate>;
}