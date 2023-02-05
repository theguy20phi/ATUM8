/**
 * @file pidFF.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides a class for a PID-FF controller (a
 * proportional, integral, derivative, and feed-forward controller).
 * This can be used to drive many different systems from their current state
 * to a desirable reference state.
 * @version 0.3
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
     * @brief A PID-FF controller. Can be used to drive
     * a variety of systems from their current state to a desired
     * reference state.
     *
     */
    class PidFF : public Controller
    {
    public:
        /**
         * @brief The parameters necessary for the
         * PidFF object to function.
         *
         */
        struct Parameters
        {
            double kP{0};
            double kI{0};
            double kD{0};
            double FF{0};
        };

        /**
         * @brief Constructs a new PidFF object
         *
         * @param kP
         * @param kI
         * @param kD
         * @param FF
         */
        PidFF(double kP = 0, double kI = 0, double kD = 0, double FF = 0);

        /**
         * @brief Constructs a new PidFF object.
         *
         * @param iParams A struct containing kP, kI, kD, and FF.
         */
        PidFF(const Parameters &iParams);

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
         * @brief Set the kP.
         *
         * @param kP
         */
        void setKP(double kP);

        /**
         * @brief Sets the kI.
         *
         * @param kI
         */
        void setKI(double kI);

        /**
         * @brief Sets the kD.
         *
         * @param kD
         */
        void setKD(double kD);

        /**
         * @brief Sets the FF.
         *
         * @param FF
         */
        void setFF(double FF);

        /**
         * @brief Get the current parameters (kP, kI, kD, and FF
         * as a struct).
         *
         * @return Parameters
         */
        Parameters getParams() const;

    private:
        void updateI(double error);
        Parameters params;
        double output{0};
        double prevError{0};
        double I{0};
    };

    using UPPidFF = std::unique_ptr<PidFF>;
    using SPPidFF = std::shared_ptr<PidFF>;
}