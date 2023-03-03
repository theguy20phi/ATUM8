/**
 * @file flywheel.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides the implementation for the flywheel.
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "atum8/constants.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/settledChecker.hpp"
#include "atum8/task.hpp"
#include "atum8/rollingAverage.hpp"
#include "atum8/slewRate.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"

namespace atum8
{
    /**
     * @brief This provides the implementation for the flywheel.
     *
     */
    class Flywheel : public Task
    {
    public:
        /**
         * @brief Constructs a new Flywheel object.
         *
         * @param iMotorGroup
         * @param iVelocityController
         * @param iVelocitySettledChecker
         * @param iRollingAverage
         * @param iSlewRate
         * @param iSpeedMultiplier
         */
        Flywheel(UPMotorGroup iMotorGroup,
                 SPController iVelocityController,
                 SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                 SPRollingAverage iRollingAverage = nullptr,
                 SPSlewRate iSlewRate = nullptr,
                 double iSpeedMultiplier = 7.0);

        /**
         * @brief This function will run in parallel to other tasks. Will be responsible
         * for maintaining flywheel velocity at reference speed.
         *
         */
        void taskFn();

        /**
         * @brief Sets the reference speed.
         *
         * @param speed
         */
        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        /**
         * @brief Gets the reference speed.
         *
         * @return okapi::QAngularSpeed
         */
        okapi::QAngularSpeed getReferenceSpeed() const;

        /**
         * @brief Gets the current speed.
         *
         * @return okapi::QAngularSpeed
         */
        okapi::QAngularSpeed getSpeed() const;

        /**
         * @brief Determines if the flywheel is ready to fire.
         *
         * @param speed
         * @return true
         * @return false
         */
        bool readyToFire(okapi::QAngularSpeed speed);

        /**
         * @brief Determines if the flywheel is ready to fire.
         *
         * @return true
         * @return false
         */
        bool readyToFire() const;

        /**
         * @brief Resets the slew rate, velocity controller, and turns off
         * the motors.
         *
         */
        void reset();

        /**
         * @brief Gets the velocity controller.
         *
         * @return SPController
         */
        SPController getVelocityController() const;

        /**
         * @brief Gets the velocity settled-checker.
         *
         * @return SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>
         */
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> getVelocitySettledChecker() const;

    private:
        UPMotorGroup motorGroup;
        double speedMultiplier{7.0};
        SPController velocityController;
        SPSlewRate slewRate;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPRollingAverage rollingAverage;
        okapi::QAngularSpeed referenceSpeed{0_degps};
    };

    using UPFlywheel = std::unique_ptr<Flywheel>;
    using SPFlywheel = std::shared_ptr<Flywheel>;

    /**
     * @brief Provides a builder for the flywheel.
     *
     */
    class SPFlywheelBuilder
    {
    public:
        /**
         * @brief Constructs the flywheel object.
         *
         * @return SPFlywheel
         */
        SPFlywheel build() const;

        /**
         * @brief Flywheel configured with these motor ports.
         *
         * @param iPorts
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withMotors(const std::vector<std::int8_t> &iPorts);

        /**
         * @brief Flywheel configured with this "speed multiplier," which is the gear ratio
         * of the flywheel (consider the motor as a green motor and a blue cartridge as a 1:3
         * ratio).
         *
         * @param iSpeedMultiplier
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withSpeedMultiplier(double iSpeedMultiplier);

        /**
         * @brief Flywheel configured with this velocity controller.
         *
         * @param iVelocityController
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withController(SPController iVelocityController);

        /**
         * @brief Flywheel configured with these parameters for the settled-checker.
         *
         * @param maxSpeedError
         * @param maxAccelError
         * @param minTime
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withSettledChecker(const okapi::QAngularSpeed &maxSpeedError,
                                             const okapi::QAngularAcceleration &maxAccelError = 0_rpmps,
                                             const okapi::QTime &minTime = 0_s);

        /**
         * @brief Flywheel configured with this size of a rolling average.
         *
         * @param size
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withRollingAverage(int size);

        /**
         * @brief Flywheel configured with these parameters for a slew rate.
         *
         * @param maxNegChange
         * @param maxPosChange
         * @return SPFlywheelBuilder
         */
        SPFlywheelBuilder withSlew(double maxNegChange, double maxPosChange);

    private:
        std::vector<std::int8_t> ports;
        double speedMultiplier{15.0};
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPRollingAverage rollingAverage;
        SPSlewRate slewRate;
    };
}