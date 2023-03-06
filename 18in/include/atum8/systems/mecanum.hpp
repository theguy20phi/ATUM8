/**
 * @file mecanum.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides the implementation for the mecanum drive.
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/settledChecker.hpp"
#include "atum8/slewRate.hpp"
#include "atum8/imus.hpp"

#include <iostream>

using namespace okapi::literals;

namespace atum8
{
    /**
     * @brief This provides the implementation for the mecanum drive.
     *
     */
    class Mecanum
    {
    public:
        /**
         * @brief The driver settings (deadzone, slew rate, stick function,
         * brake mode, max power).
         *
         */
        struct DriverSettings
        {
            int deadZone{0};
            SPSlewRate forwardSlewRate{nullptr};
            SPSlewRate strafeSlewRate{nullptr};
            SPSlewRate turnSlewRate{nullptr};
            std::function<int(int)> stickFunction = {[](int input)
                                                     { return input; }};
            pros::motor_brake_mode_e brakeMode{pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST};
            double maxPower{1.0};
        };

        using UPDriverSettings = std::unique_ptr<DriverSettings>;
        using SPDriverSettings = std::shared_ptr<DriverSettings>;

        /**
         * @brief The necessary dimensions for autonomous.
         *
         */
        struct Dimensions
        {
            okapi::QLength baseWidth;
            okapi::QLength wheelCircum;
        };

        using UPDimensions = std::unique_ptr<Dimensions>;
        using SPDimensions = std::shared_ptr<Dimensions>;

        /**
         * @brief Constructs a new Mecanum object.
         *
         * @param iRFMotor
         * @param iLFMotor
         * @param iLBMotor
         * @param iRBMotor
         * @param iDimensions
         * @param iDriverSettings
         * @param iForwardController
         * @param iTurnController
         * @param iforwardSettledChecker
         * @param iTurnSettledChecker
         * @param iForwardSlewRate
         * @param iTurnSlewRate
         * @param iImus
         * @param iImuTrust
         */
        Mecanum(UPMotor iRFMotor,
                UPMotor iLFMotor,
                UPMotor iLBMotor,
                UPMotor iRBMotor,
                SPDimensions iDimensions,
                SPDriverSettings iDriverSettings,
                SPController iForwardController,
                SPController iTurnController,
                SPSettledChecker<okapi::QLength, okapi::QSpeed> iforwardSettledChecker,
                SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                SPSlewRate iForwardSlewRate,
                SPSlewRate iTurnSlewRate,
                UPImus iImus = nullptr,
                double iImuTrust = 0.5);

        /**
         * @brief The driver controls for the drive.
         *
         * @param forward
         * @param strafe
         * @param turn
         */
        void driver(int forward = 0, int strafe = 0, int turn = 0);

        /**
         * @brief Drives the robot forward a certain distance, with a timeout, and an
         * output limit. Negative values indicated going backwards.
         *
         * @param distance
         * @param maxTime
         * @param maxForward
         */
        void forward(const okapi::QLength &distance, const okapi::QTime &maxTime = 0_s, int maxForward = 127);

        /**
         * @brief Turns the robot a certain angle, with a timeout, and an output limit.
         * Negative values indicated a counter-clockwise turn.
         *
         * @param angle
         * @param maxTime
         * @param maxTurn
         */
        void turn(const okapi::QAngle &angle, const okapi::QTime &maxTime = 0_s, int maxTurn = 127);

        /**
         * @brief Applies voltages to the appropriate motors given values for forward, strafe, and turn.
         *
         * @param forward
         * @param strafe
         * @param turn
         */
        void move(int forward = 0, int strafe = 0, int turn = 0);

        /**
         * @brief Gets the distance the drive has traveled since last tared.
         *
         * @return okapi::QLength
         */
        okapi::QLength getDistance() const;

        /**
         * @brief Gets the angle the drive has turned since last tared.
         *
         * @return okapi::QAngle
         */
        okapi::QAngle getAngle() const;

        /**
         * @brief Determines if the drive has settled.
         *
         * @param distanceError
         * @param angleError
         * @return true
         * @return false
         */
        bool isSettled(const okapi::QLength &distanceError, const okapi::QAngle &angleError);

        /**
         * @brief Resets the drive and calibrates the IMUs. Because it calibrates the IMUs,
         * it blocks!
         *
         */
        void reset();

        /**
         * @brief Resets the sensor values of t
         *
         */
        void tare();

        /**
         * @brief Sets the brake mode of the drive; does not change driver settings!
         *
         * @param brakeMode
         */
        void setBrakeMode(const pros::motor_brake_mode_e &brakeMode);

        /**
         * @brief Gets the driver settings.
         *
         * @return SPDriverSettings
         */
        SPDriverSettings getDriverSettings() const;

    private:
        void applyBrakes();
        bool isTimeNotExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        void toReference(const std::function<okapi::QLength()> &distanceError,
                         const std::function<okapi::QAngle()> &angleError,
                         const okapi::QTime &maxTime,
                         int maxForward = 127,
                         int maxTurn = 127);
        int useForwardController(const okapi::QLength &distanceError, int maxForward = 127);
        int useTurnController(const okapi::QAngle &angleError, int maxTurn = 127);
        UPMotor rFMotor;
        UPMotor lFMotor;
        UPMotor lBMotor;
        UPMotor rBMotor;
        SPDimensions dimensions;
        SPDriverSettings driverSettings;
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        UPImus imus;
        double imuTrust{0.5};
    };

    using UPMecanum = std::unique_ptr<Mecanum>;
    using SPMecanum = std::shared_ptr<Mecanum>;

    /**
     * @brief Provides a builder for the mecanum drive.
     *
     */
    class SPMecanumBuilder
    {
    public:
        /**
         * @brief Constructs the mecanum object.
         *
         * @return SPMecanum
         */
        SPMecanum build() const;

        /**
         * @brief Mecanum configured with this right-front motor.
         *
         * @param port
         * @param gearset
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withRFMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        /**
         * @brief Mecanum configured with this left-front motor.
         *
         * @param port
         * @param gearset
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withLFMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        /**
         * @brief Mecanum configured with this left-back motor.
         *
         * @param port
         * @param gearset
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withLBMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        /**
         * @brief Mecanum configured with this right-back motor.
         *
         * @param port
         * @param gearset
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withRBMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        /**
         * @brief Mecanum configured with this base width (from wheel-to-wheel width-wise).
         *
         * @param baseWidth
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withBaseWidth(const okapi::QLength &baseWidth);

        /**
         * @brief Mecanum configured with this wheel circumference.
         *
         * @param wheelCircum
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withWheelCircum(const okapi::QLength &wheelCircum);

        /**
         * @brief Mecanum configured with this stick deadzone.
         *
         * @param deadZone
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withStickDeadZone(int deadZone);

        /**
         * @brief Mecanum configured with this stick slew.
         *
         * @param slew
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder witStickSlew(double slew);

        /**
         * @brief Mecanum configured with this stick function (function to apply to the
         * stick inputs).
         *
         * @param stickFunction
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withStickFunction(const std::function<int(int)> &stickFunction);

        /**
         * @brief Mecanum configured with this stick max (proportion of max power to apply
         * to the drive).
         *
         * @param maxPower
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withStickMax(int maxPower);

        /**
         * @brief Mecanum configured with this forward controller.
         *
         * @param iForwardController
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withForwardController(SPController iForwardController);

        /**
         * @brief Mecanum configured with this turn controller.
         *
         * @param iTurnController
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withTurnController(SPController iTurnController);

        /**
         * @brief Mecanum configured with these parameters for forward settled checker.
         *
         * @param distance
         * @param speed
         * @param time
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withForwardSettledChecker(const okapi::QLength &distance,
                                                   const okapi::QSpeed &speed = 0_inps,
                                                   const okapi::QTime &time = 0_s);

        /**
         * @brief Mecanum configured with these parameters for turn settled checker.
         *
         * @param angle
         * @param angularSpeed
         * @param time
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withTurnSettledChecker(const okapi::QAngle &angle,
                                                const okapi::QAngularSpeed &angularSpeed,
                                                const okapi::QTime &time);

        /**
         * @brief Mecanum configured with this forward slew rate.
         *
         * @param slewRate
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withForwardSlew(double slewRate);

        /**
         * @brief Mecanum configured with this turn slew rate.
         *
         * @param slewRate
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withTurnSlew(double slewRate);

        /**
         * @brief Mecanum configured with these ports for IMUs and this trust.
         *
         * @param ports
         * @param trust
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withImus(const std::vector<int> &ports, double trust = 0.5);

        /**
         * @brief Mecanum configured with this default brake mode.
         *
         * @param brakeMode
         * @return SPMecanumBuilder
         */
        SPMecanumBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

    private:
        int rFPort;
        pros::motor_gearset_e_t rFGearset;
        int lFPort;
        pros::motor_gearset_e_t lFGearset;
        int lBPort;
        pros::motor_gearset_e_t lBGearset;
        int rBPort;
        pros::motor_gearset_e_t rBGearset;
        Mecanum::SPDimensions dimensions{std::make_shared<Mecanum::Dimensions>()};
        Mecanum::SPDriverSettings driverSettings{std::make_shared<Mecanum::DriverSettings>()};
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        std::vector<int> imuPorts;
        double imuTrust{0.5};
    };
}