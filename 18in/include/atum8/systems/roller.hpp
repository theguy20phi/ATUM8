/**
 * @file roller.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides the implementation for the roller.
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "atum8/misc/constants.hpp"
#include "pros/misc.hpp"

namespace atum8
{
    /**
     * @brief Provides the implementation for the roller.
     *
     */
    class Roller
    {
    public:
        /**
         * @brief Constructs a new Roller object.
         *
         * @param iMotor
         */
        Roller(UPMotor iMotor);

        /**
         * @brief Provides the controls for the driver controller portion.
         * 
         * @param master 
         */
        void control(pros::Controller master);

        /**
         * @brief Runs the roller at a given speed.
         *
         * @param speed
         */
        void runRoller(int speed = 200);

        /**
         * @brief Stops the roller.
         *
         */
        void stopRoller();

        /**
         * @brief Runs the roller to a certain position, at a certain speed.
         *
         * @param position
         * @param speed
         */
        void runForAt(double position, int speed = 100);

    private:
        UPMotor motor;
    };

    using UPRoller = std::unique_ptr<Roller>;
    using SPRoller = std::shared_ptr<Roller>;

    /**
     * @brief Provides a builder for the roller.
     *
     */
    class SPRollerBuilder
    {
    public:
        /**
         * @brief Constructs the roller object.
         *
         * @return SPRoller
         */
        SPRoller build() const;

        /**
         * @brief Roller configured with this motor.
         *
         * @param iPort
         * @param iGearset
         * @return SPRollerBuilder
         */
        SPRollerBuilder withMotor(int iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
    };
}