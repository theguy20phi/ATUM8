/**
 * @file slider.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides a controller that slides between the output
 * of two controllers based upon the difference between the current
 * state and desired state (i.e. error). This allows for a smooth transition
 * between two controllers intended for far distances and short distances.
 * @version 0.2
 * @date 2023-02-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "controller.hpp"
#include <cmath>

namespace atum8
{
    /**
     * @brief A controller that's output is a weighted average between the
     * values provided by two other controllers. The weighted average uses a
     * modified hyperbolic tangent and places more weight upon the
     * "far" controller when error is larger and more weight on the "close"
     * controller when error is smaller.
     *
     */
    class Slider : public Controller
    {
    public:
        /**
         * @brief Construct a new slider object, given two controllers
         * and the value for error at which there is equal weight placed
         * upon both, and how "quickly" weight changes with increasing
         * error to the far controller.
         *
         * @param iCloseController The controller that is preferred when error is low.
         * @param iFarController The controller that is preferred when error is high.
         * @param iShiftLocation The value of error where the weight for both controllers
         * is equal.
         * @param iShiftSpeed How quickly weight should increase for the second
         * controller when near the shift location (lower values are a smooth transition
         * between the close controller and far controller, higher values are a more
         * abrupt transition).
         */
        Slider(UPController iCloseController,
               UPController iFarController,
               double iShiftLocation,
               double iShiftSpeed);

        /**
         * @brief Gets the output given the current state and desire reference
         * state.
         *
         * @param state The current state.
         * @param reference The desired reference state.
         * @return double 
         */
        double getOutput(double state, double reference);

        /**
         * @brief Gets the output given the difference between the current state
         * and desired reference state, error.
         *
         * @param error The difference between the current state and the desired
         * reference state.
         * @return double
         */
        double getOutput(double error);

    private:
        double weight(double error) const;
        UPController closeController;
        UPController farController;
        double shiftLocation{0};
        double shiftSpeed{0};
        double output{0};
    };

    using UPSlider = std::unique_ptr<Slider>;
    using SPSlider = std::shared_ptr<Slider>;
}