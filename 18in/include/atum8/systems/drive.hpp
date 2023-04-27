#pragma once

#include "atum8/controllers/controller.hpp"
#include "atum8/misc/settledChecker.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/devices/poseEstimator.hpp"
#include "atum8/filters/filter.hpp"
#include "atum8/gui/autonSelector.hpp"
#include "pros/misc.hpp"
#include <numeric>

using namespace okapi::literals;

namespace atum8
{
    class Drive
    {
    public:
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

        Drive(UPMotorGroup iLeft,
              UPMotorGroup iRight,
              SPPoseEstimator iPoseEstimator,
              UPVision iVision,
              SPDriverSettings iDriverSettings,
              SPAutonSelector iAutonSelector,
              SPController iLateralController,
              SPController iAngularController,
              SPController iAimController,
              SPLateralSettledChecker iLateralSettledChecker,
              SPAngularSettledChecker iAngularSettledChecker,
              SPFilter iAimFilter);

        void control(pros::Controller master);

        void moveTo(Position target,
                    const okapi::QTime &maxTime = 0_s,
                    bool reversed = false,
                    int maxLateral = 127,
                    int maxAngular = 127,
                    const okapi::QLength &offset = 0_in);

        void pointAt(Position target,
                     const okapi::QTime &maxTime = 0_s,
                     bool reversed = false,
                     bool useVision = false,
                     int maxAngular = 127);

        double visionAim();

        void move(int forward = 0, int turn = 0);

        void tare();

        void setColor(const Color &iColor);

        void setBrakeMode(const pros::motor_brake_mode_e &brakeMode);

        SPDriverSettings getDriverSettings() const;

    private:
        void driverMove(int leftInput = 0, int rightInput = 0);
        void applyBrakes();
        bool isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        bool isSettled(const okapi::QLength &lateralError);
        double getVisionAimError();
        UPMotorGroup left;
        UPMotorGroup right;
        SPPoseEstimator poseEstimator;
        UPVision vision;
        SPDriverSettings driverSettings;
        SPAutonSelector autonSelector;
        SPController lateralController;
        SPController angularController;
        SPController aimController;
        SPLateralSettledChecker lateralSettledChecker;
        SPAngularSettledChecker angularSettledChecker;
        SPFilter aimFilter{std::make_shared<Filter>()};
        Color color{Color::Red};
        pros::vision_signature redSig;
        pros::vision_signature blueSig;
    };

    using UPDrive = std::unique_ptr<Drive>;
    using SPDrive = std::shared_ptr<Drive>;

    class SPDriveBuilder
    {
    public:
        SPDrive build() const;

        SPDriveBuilder withLeftPorts(const std::vector<int8_t> &iLeftPorts);

        SPDriveBuilder withRightPorts(const std::vector<int8_t> &iRightPorts);

        SPDriveBuilder withPoseEstimator(SPPoseEstimator iPoseEstimator);

        SPDriveBuilder withVision(int8_t port);

        SPDriveBuilder withStickDeadZone(int deadZone);

        SPDriveBuilder withStickFunction(const std::function<int(int)> &stickFunction);

        SPDriveBuilder withStickMax(int maxPower);

        SPDriveBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

        SPDriveBuilder withAutonSelector(SPAutonSelector iAutonSelector);

        SPDriveBuilder withLateralController(SPController iLateralController);

        SPDriveBuilder withAngularController(SPController iAngularController);

        SPDriveBuilder withAimController(SPController iAimController);

        SPDriveBuilder withLateralSettledChecker(const okapi::QLength &distance,
                                                 const okapi::QSpeed &speed = 0_inps,
                                                 const okapi::QTime &time = 0_s);

        SPDriveBuilder withAngularSettledChecker(const okapi::QAngle &angle,
                                                 const okapi::QAngularSpeed &angularSpeed = 0_rpm,
                                                 const okapi::QTime &time = 0_s);

        SPDriveBuilder withAimFilter(SPFilter iAimFilter);

    private:
        std::vector<int8_t> leftPorts;
        std::vector<int8_t> rightPorts;
        SPPoseEstimator poseEstimator;
        int8_t visionPort;
        Drive::SPDriverSettings driverSettings{std::make_shared<Drive::DriverSettings>()};
        SPAutonSelector autonSelector;
        SPController lateralController;
        SPController angularController;
        SPController aimController;
        SPLateralSettledChecker lateralSettledChecker;
        SPAngularSettledChecker angularSettledChecker;
        SPFilter aimFilter{std::make_shared<Filter>()};
    };
}