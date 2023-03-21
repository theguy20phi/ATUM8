/**
 * @file controller.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides the Controller interface that will be
 * used to make controllers easily swappable in control algorithms.
 * @version 0.5
 * @date 2023-02-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <memory>
#include "atum8/misc/constants.hpp"
#include "pros/rtos.hpp"

namespace atum8
{
    /**
     * @brief Interface for controllers: makes PID, Take-Back-Half,
     * Bang-Bang, and the like, swappable with one another.
     *
     */
    class Controller
    {
    public:
        /**
         * @brief Get the output of the controller when given
         * the current state and desired state.
         *
         * @param state The current state.
         * @param reference The desired state.
         * @return double The output of the controller.
         */
        virtual double getOutput(double state, double reference) = 0;

        /**
         * @brief Get the output when given the error.
         *
         * @param error Difference between the current and desired state.
         * @return double The output of the controller.
         */
        virtual double getOutput(double error) = 0;

        /**
         * @brief Get the output of the controller last calculated.
         *
         * @return double The output of the controller;
         */
        virtual double getOutput() const;

        /**
         * @brief Resets the controller by setting output to 0.
         * 
         */
        virtual void reset();

    protected:
        bool sampleTimePassed();
        double output{0};

    private:
        long unsigned int prevTime{0};
    };

    using UPController = std::unique_ptr<Controller>;
    using SPController = std::shared_ptr<Controller>;
}