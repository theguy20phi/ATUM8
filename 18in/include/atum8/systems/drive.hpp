/**
 * @file drive.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides the implementation for the drive.
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
#include "atum8/misc/settledChecker.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/imus.hpp"
#include <numeric>

using namespace okapi::literals;

namespace atum8
{
    /**
     * @brief This provides the implementation for the Drive drive.
     *
     */
    class Drive
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
         * @brief Constructs a new Drive object.
         *
         * @param iLeft
         * @param iRight
         * @param iGearing
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
        Drive(UPMotorGroup iLeft,
              UPMotorGroup iRight,
              double iGearing,
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
         * @param turn
         */
        void driver(int forward = 0, int turn = 0);

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
         * @brief Applies voltages to the appropriate motors given values for forward and turn.
         *
         * @param forward
         * @param turn
         */
        void move(int forward = 0, int turn = 0);

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
        UPMotorGroup left;
        UPMotorGroup right;
        double gearing;
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

    using UPDrive = std::unique_ptr<Drive>;
    using SPDrive = std::shared_ptr<Drive>;

    /**
     * @brief Provides a builder for the Drive drive.
     *
     */
    class SPDriveBuilder
    {
    public:
        /**
         * @brief Constructs the Drive object.
         *
         * @return SPDrive
         */
        SPDrive build() const;

        /**
         * @brief Drive configured with these left motor ports.
         *
         * @param iLeftPorts
         * @return SPDriveBuilder
         */
        SPDriveBuilder withLeftPorts(const std::vector<std::int8_t> &iLeftPorts);

        /**
         * @brief Drive configured with these right motor ports.
         *
         * @param iRightPorts
         * @return SPDriveBuilder
         */
        SPDriveBuilder withRightPorts(const std::vector<std::int8_t> &iRightPorts);

        /**
         * @brief Drive configured with this gearing,  which is the gear ratio
         * of the drive (consider the motor as a green motor and a blue cartridge as a 1:3
         * ratio; for a blue cartridge, iGearing would equal 3.0).
         *
         * @param iGearing
         * @return SPDriveBuilder
         */
        SPDriveBuilder withGearing(double iGearing);

        /**
         * @brief Drive configured with this base width (from wheel-to-wheel width-wise).
         *
         * @param baseWidth
         * @return SPDriveBuilder
         */
        SPDriveBuilder withBaseWidth(const okapi::QLength &baseWidth);

        /**
         * @brief Drive configured with this wheel circumference.
         *
         * @param wheelCircum
         * @return SPDriveBuilder
         */
        SPDriveBuilder withWheelCircum(const okapi::QLength &wheelCircum);

        /**
         * @brief Drive configured with this stick deadzone.
         *
         * @param deadZone
         * @return SPDriveBuilder
         */
        SPDriveBuilder withStickDeadZone(int deadZone);

        /**
         * @brief Drive configured with this stick slew.
         *
         * @param slew
         * @return SPDriveBuilder
         */
        SPDriveBuilder witStickSlew(double slew);

        /**
         * @brief Drive configured with this stick function (function to apply to the
         * stick inputs).
         *
         * @param stickFunction
         * @return SPDriveBuilder
         */
        SPDriveBuilder withStickFunction(const std::function<int(int)> &stickFunction);

        /**
         * @brief Drive configured with this stick max (proportion of max power to apply
         * to the drive).
         *
         * @param maxPower
         * @return SPDriveBuilder
         */
        SPDriveBuilder withStickMax(int maxPower);

        /**
         * @brief Drive configured with this forward controller.
         *
         * @param iForwardController
         * @return SPDriveBuilder
         */
        SPDriveBuilder withForwardController(SPController iForwardController);

        /**
         * @brief Drive configured with this turn controller.
         *
         * @param iTurnController
         * @return SPDriveBuilder
         */
        SPDriveBuilder withTurnController(SPController iTurnController);

        /**
         * @brief Drive configured with these parameters for forward settled checker.
         *
         * @param distance
         * @param speed
         * @param time
         * @return SPDriveBuilder
         */
        SPDriveBuilder withForwardSettledChecker(const okapi::QLength &distance,
                                                 const okapi::QSpeed &speed = 0_inps,
                                                 const okapi::QTime &time = 0_s);

        /**
         * @brief Drive configured with these parameters for turn settled checker.
         *
         * @param angle
         * @param angularSpeed
         * @param time
         * @return SPDriveBuilder
         */
        SPDriveBuilder withTurnSettledChecker(const okapi::QAngle &angle,
                                              const okapi::QAngularSpeed &angularSpeed,
                                              const okapi::QTime &time);

        /**
         * @brief Drive configured with this forward slew rate.
         *
         * @param slewRate
         * @return SPDriveBuilder
         */
        SPDriveBuilder withForwardSlew(double slewRate);

        /**
         * @brief Drive configured with this turn slew rate.
         *
         * @param slewRate
         * @return SPDriveBuilder
         */
        SPDriveBuilder withTurnSlew(double slewRate);

        /**
         * @brief Drive configured with these ports for IMUs and this trust.
         *
         * @param ports
         * @param trust
         * @return SPDriveBuilder
         */
        SPDriveBuilder withImus(const std::vector<int> &ports, double trust = 0.5);

        /**
         * @brief Drive configured with this default brake mode.
         *
         * @param brakeMode
         * @return SPDriveBuilder
         */
        SPDriveBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

    private:
        std::vector<int8_t> leftPorts;
        std::vector<int8_t> rightPorts;
        double gearing;
        Drive::SPDimensions dimensions{std::make_shared<Drive::Dimensions>()};
        Drive::SPDriverSettings driverSettings{std::make_shared<Drive::DriverSettings>()};
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