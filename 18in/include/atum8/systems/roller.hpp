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

#include "atum8/constants.hpp"
#include "atum8/task.hpp"

namespace atum8
{
    /**
     * @brief Provides the implementation for the roller.
     *
     */
    class Roller : public Task
    {
    public:
        /**
         * @brief Constructs a new Roller object.
         *
         * @param iMotor
         * @param iOptical
         * @param iColor
         */
        Roller(UPMotor iMotor, UPOptical iOptical, const Color &iColor = Color::Red);

        /**
         * @brief This function will be ran in parallel to other tasks. It is responsible
         * for turning the roller to the appropriate alliance color when requested.
         *
         */
        void taskFn();

        /**
         * @brief Runs the roller at a given speed.
         *
         * @param speed
         */
        void runRoller(int speed = 127);

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
        void runForAt(double position, int speed = 50);

        /**
         * @brief Requests the roller to turn to the appropriate alliance color.
         *
         */
        void turnToColor();

        /**
         * @brief Sets the appropriate alliance color.
         *
         * @param iColor
         */
        void setColor(const Color &iColor);

        /**
         * @brief Gets the alliance color.
         *
         * @return Color
         */
        Color getColor() const;

    private:
        std::function<bool()> readWrongColor();
        std::function<bool()> readRightColor();
        UPMotor motor;
        UPOptical optical;
        Color color{Color::Red};
        bool turningToColor{false};
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

        /**
         * @brief Roller configured with this optical port.
         *
         * @param iPort
         * @return SPRollerBuilder
         */
        SPRollerBuilder withOptical(int iPort);

        /**
         * @brief Roller configured with this alliance color.
         *
         * @param iColor
         * @return SPRollerBuilder
         */
        SPRollerBuilder withColor(const Color &iColor);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
        int opticalPort;
        Color color;
    };
}