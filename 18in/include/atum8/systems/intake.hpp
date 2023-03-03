/**
 * @file intake.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides the implementation for the intake.
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "atum8/constants.hpp"
#include "atum8/task.hpp"
#include "flywheel.hpp"

namespace atum8
{
    /**
     * @brief This provides the implementation for the intake.
     *
     */
    class Intake : public Task
    {
    public:
        /**
         * @brief Constructs a new Intake object.
         *
         * @param iMotor
         * @param iPiston
         * @param iFlywheel
         * @param iShotDelay
         */
        Intake(UPMotor iMotor,
               UPADIDigitalOut iPiston,
               const SPFlywheel &iFlywheel,
               int iShotDelay = 500);

        /**
         * @brief This function will be ran in parallel to other tasks.
         * It is responsible for firing the intake as necessary.
         *
         */
        void taskFn();

        /**
         * @brief Runs the intake at a speed.
         *
         * @param speed
         */
        void runIntake(int speed = 127);

        /**
         * @brief Stops the intake.
         *
         */
        void stopIntake();

        /**
         * @brief Shoots the intake.
         *
         * @param iShooting
         * @param iFlywheelBlocks
         */
        void shoot(int iShooting = 1, bool iFlywheelBlocks = true);

        /**
         * @brief Determines if the intake is shooting.
         *
         * @return true
         * @return false
         */
        bool isShooting() const;

    private:
        bool shouldShoot();
        UPMotor motor;
        UPADIDigitalOut piston;
        SPFlywheel flywheel;
        const int shotDelay;
        int shooting{0};
        bool flywheelBlocks{true};
    };

    using UPIntake = std::unique_ptr<Intake>;
    using SPIntake = std::shared_ptr<Intake>;

    /**
     * @brief Provides a builder for the intake.
     *
     */
    class SPIntakeBuilder
    {
    public:
        /**
         * @brief Constructs the intake object.
         *
         * @return SPIntake
         */
        SPIntake build() const;

        /**
         * @brief Intake configured with these motors.
         *
         * @param iPort
         * @param iGearset
         * @return SPIntakeBuilder
         */
        SPIntakeBuilder withMotor(int iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        /**
         * @brief Intake configured with this piston port.
         *
         * @param iPort
         * @return SPIntakeBuilder
         */
        SPIntakeBuilder withPiston(int iPort);

        /**
         * @brief Intake configured with this flywheel reference.
         *
         * @param iFlywheel
         * @return SPIntakeBuilder
         */
        SPIntakeBuilder withFlywheel(const SPFlywheel &iFlywheel);

        /**
         * @brief Intake configured with this shot delay (in ms).
         *
         * @param iShotDelay
         * @return SPIntakeBuilder
         */
        SPIntakeBuilder withShotDelay(int iShotDelay = 500);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
        int pistonPort;
        SPFlywheel flywheel;
        int shotDelay;
    };
}