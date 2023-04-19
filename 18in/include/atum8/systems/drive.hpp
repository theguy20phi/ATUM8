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

#include "atum8/controllers/controller.hpp"
#include "atum8/misc/settledChecker.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/devices/poseEstimator.hpp"
#include "atum8/filters/filter.hpp"
#include "pros/misc.hpp"
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
            std::function<int(int)> stickFunction = {[](int input)
                                                     { return input; }};
            pros::motor_brake_mode_e brakeMode{pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST};
            double maxPower{1.0};
        };

        using UPDriverSettings = std::unique_ptr<DriverSettings>;
        using SPDriverSettings = std::shared_ptr<DriverSettings>;

        /**
         * @brief Constructs a new Drive object.
         *
         * @param iLeft
         * @param iRight
         * @param iPoseEstimator
         * @param iVision
         * @param iDriverSettings
         * @param iLateralController
         * @param iAngularController
         * @param iAimController
         * @param iFinalLateralSettledChecker
         * @param iFinalAngularSettledChecker
         * @param iMidwayLateralSettledChecker
         * @param iMidwayAngularSettledChecker
         * @param iAimFilter
         */
        Drive(UPMotorGroup iLeft,
              UPMotorGroup iRight,
              SPPoseEstimator iPoseEstimator,
              UPVision iVision,
              SPDriverSettings iDriverSettings,
              SPController iLateralController,
              SPController iAngularController,
              SPController iAimController,
              SPLateralSettledChecker iFinalLateralSettledChecker,
              SPLateralSettledChecker iMidwayLateralSettledChecker,
              SPAngularSettledChecker iAngularSettledChecker,
              SPFilter iAimFilter);

        /**
         * @brief Provides the controls for the driver controlled portion of the match.
         *
         * @param master
         */
        void control(pros::Controller master);

        void moveTo(const std::vector<Position> &positions,
                    const okapi::QTime &maxTime = 0_s,
                    int maxLateral = 127,
                    int maxAngular = 127);

        /**
         * @brief Returns output for aim assist (to be added to left side and subtracted
         * from the right).
         * 
         * @return double 
         */
        double visionAim();

        /**
         * @brief Applies voltages to the appropriate motors given values for forward and turn.
         *
         * @param leftInput
         * @param rightInput
         */
        void move(int leftInput = 0, int rightInput = 0);

        /**
         * @brief Resets the sensor values.
         *
         */
        void tare();

        /**
         * @brief Sets the color of the alliance.
         * 
         * @param iColor 
         */
        void setColor(const Color &iColor);

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
        void driverMove(int leftInput = 0, int rightInput = 0);
        void applyBrakes();
        bool isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        bool isSettled(const okapi::QLength &lateralError,
                       const okapi::QAngle &angularError,
                       int currentIndex,
                       int lastIndex);
        Position generateWaypoint(const Position &state, const Position &endPoint);
        UPMotorGroup left;
        UPMotorGroup right;
        SPPoseEstimator poseEstimator;
        UPVision vision;
        SPDriverSettings driverSettings;
        SPController lateralController;
        SPController angularController;
        SPController aimController;
        SPLateralSettledChecker finalLateralSettledChecker;
        SPLateralSettledChecker midwayLateralSettledChecker;
        SPAngularSettledChecker angularSettledChecker;
        SPFilter aimFilter{std::make_shared<Filter>()};
        Color color{Color::Red};
        pros::vision_signature redSig;
        pros::vision_signature blueSig;
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
        SPDriveBuilder withLeftPorts(const std::vector<int8_t> &iLeftPorts);

        /**
         * @brief Drive configured with these right motor ports.
         *
         * @param iRightPorts
         * @return SPDriveBuilder
         */
        SPDriveBuilder withRightPorts(const std::vector<int8_t> &iRightPorts);

        /**
         * @brief Drive configured with this pose estimator.
         *
         * @param iPoseEstimator
         * @return SPDriveBuilder
         */
        SPDriveBuilder withPoseEstimator(SPPoseEstimator iPoseEstimator);

        /**
         * @brief Drive configured with this vision sensor.
         *
         * @param port
         * @return SPDriveBuilder
         */
        SPDriveBuilder withVision(int8_t port);

        /**
         * @brief Drive configured with this stick deadzone.
         *
         * @param deadZone
         * @return SPDriveBuilder
         */
        SPDriveBuilder withStickDeadZone(int deadZone);

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
         * @brief Drive configured with this default brake mode.
         *
         * @param brakeMode
         * @return SPDriveBuilder
         */
        SPDriveBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

        SPDriveBuilder withLateralController(SPController iLateralController);

        SPDriveBuilder withAngularController(SPController iAngularController);

        /**
         * @brief Drive configured with this aim controller.
         *
         * @param iAimController
         * @return SPDriveBuilder
         */
        SPDriveBuilder withAimController(SPController iAimController);

        SPDriveBuilder withFinalLateralSettledChecker(const okapi::QLength &distance,
                                                      const okapi::QSpeed &speed = 0_inps,
                                                      const okapi::QTime &time = 0_s);

        SPDriveBuilder withMidwayLateralSettledChecker(const okapi::QLength &distance);

        SPDriveBuilder withAngularSettledChecker(const okapi::QAngle &angle,
                                                 const okapi::QAngularSpeed &angularSpeed = 0_rpm,
                                                 const okapi::QTime &time = 0_s);

        /**
         * @brief Drive configured with this aim filter.
         * 
         * @param iAimFilter 
         * @return SPDriveBuilder 
         */
        SPDriveBuilder withAimFilter(SPFilter iAimFilter);

    private:
        std::vector<int8_t> leftPorts;
        std::vector<int8_t> rightPorts;
        SPPoseEstimator poseEstimator;
        int8_t visionPort;
        Drive::SPDriverSettings driverSettings{std::make_shared<Drive::DriverSettings>()};
        SPController lateralController;
        SPController angularController;
        SPController aimController;
        SPLateralSettledChecker finalLateralSettledChecker;
        SPLateralSettledChecker midwayLateralSettledChecker;
        SPAngularSettledChecker angularSettledChecker;
        SPFilter aimFilter{std::make_shared<Filter>()};
    };
}