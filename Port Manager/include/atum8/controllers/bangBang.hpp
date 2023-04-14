/**
 * @file bangBang.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides an implementation of the simple Bang-Bang controller.
 * The bang-bang controller maps error values to a finite number of
 * output values. Simple, but may be useful in certain applications.
 * @version 0.1
 * @date 2023-02-05
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include "controller.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace atum8
{
    /**
     * @brief The bang-bang controller maps error values to a finite number of
     * output values. Simple, but may be useful in certain applications.
     *
     */
    class BangBang : public Controller
    {
    public:
        using ErrorOutputPair = std::pair<double, double>;
        using ErrorOutputPairs = std::vector<ErrorOutputPair>;

        /**
         * @brief Constructs a new BangBang object
         *
         * @param iErrorOutputPairs
         * @param iMaxOutput
         */
        BangBang(ErrorOutputPairs iErrorOutputPairs, double iMaxOutput);

        /**
         * @brief Get the output based on the current state and the current
         * desirable reference state.
         *
         * @param state The current state of the system.
         * @param reference The current desirable reference state.
         * @return double
         */
        double getOutput(double state, double reference);

        /**
         * @brief Get the output based on the difference between the current state
         * and the current desirable reference state.
         *
         * @param error The difference between the current state and the current desirable
         * reference state.
         * @return double
         */
        double getOutput(double error);

    private:
        const double maxOutput;
        ErrorOutputPairs errorOutputPairs;
    };

    using UPBangBang = std::unique_ptr<BangBang>;
    using SPBangBang = std::shared_ptr<BangBang>;
}