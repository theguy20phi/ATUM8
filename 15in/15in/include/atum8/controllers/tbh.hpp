/**
 * @file tbh.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides an implementation for the Take-Back-Half control
 * algorithm, which is similar to an integrator, but with additional logic
 * whenever the state crosses the reference. Useful for velocity control.
 * @version 0.1
 * @date 2023-02-05
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
     * @brief Provides an implementation for the Take-Back-Half control
     * algorithm, which is similar to an integrator, but with additional logic
     * whenever the state crosses the reference. Useful for velocity control.
     *
     */
    class Tbh : public Controller
    {
    public:
        /**
         * @brief Constructs a new Tbh object
         *
         * @param iKTbh Functions similar to kP from a PID controller.
         */
        Tbh(double iKTbh);

        /**
         * @brief Gets the output given the current state and reference.
         *
         * @param state The current state of the system.
         * @param reference The desired reference state of the system.
         * @return double
         */
        double getOutput(double state, double reference);

        /**
         * @brief Gets the output given the current difference between the state
         * and the desired reference state (i.e. error).
         *
         * @param error The difference between the current state and the desired
         * reference state.
         * @return double
         */
        double getOutput(double error);

        /**
         * @brief Sets the kTbh.
         *
         * @param iKTbh
         */
        void setKTbh(double iKTbh);

        /**
         * @brief Gets the kTbh.
         *
         * @return double
         */
        double getKTbh() const;

    private:
        double feedForward{0};
        double kTbh{0};
        double prevError{0};
    };

    using UPTbh = std::unique_ptr<Tbh>;
    using SPTbh = std::shared_ptr<Tbh>;
}