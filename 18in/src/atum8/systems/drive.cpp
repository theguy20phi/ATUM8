#include "drive.hpp"

using namespace okapi::literals;

namespace atum8
{
    /* -------------------------------------------------------------------------- */
    /*                                   Drive                                  */
    /* -------------------------------------------------------------------------- */

    Drive::Drive(UPMotorGroup iLeft,
                 UPMotorGroup iRight,
                 SPPoseEstimator iPoseEstimator,
                 SPDriverSettings iDriverSettings,
                 SPController iLateralController,
                 SPController iAngularController,
                 SPLateralSettledChecker iFinalLateralSettledChecker,
                 SPLateralSettledChecker iMidwayLateralSettledChecker,
                 SPAngularSettledChecker iAngularSettledChecker) : left{std::move(iLeft)},
                                                                   right{std::move(iRight)},
                                                                   poseEstimator{iPoseEstimator},
                                                                   driverSettings{iDriverSettings},
                                                                   lateralController{iLateralController},
                                                                   angularController{iAngularController},
                                                                   finalLateralSettledChecker{iFinalLateralSettledChecker},
                                                                   midwayLateralSettledChecker{iMidwayLateralSettledChecker},
                                                                   angularSettledChecker{iAngularSettledChecker}
    {
    }

    void Drive::moveTo(const std::vector<Position> &positions,
                       const okapi::QTime &maxTime,
                       int maxLateral,
                       int maxAngular)
    {
        tare();
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        for (int i{0}; i < positions.size(); i++)
        {
            while (!isTimeExpired(startTime, maxTime))
            {
                const Position state{poseEstimator->getPosition()};
                const Position waypoint{generateWaypoint(state, positions[i])};
                okapi::QLength lateralError{distance(state, waypoint)};
                okapi::QAngle angularError{angle(state, waypoint)};
                if (okapi::abs(angularError) > 90_deg) {
                    lateralError = -1 * lateralError;
                    angularError = okapi::OdomMath::constrainAngle180(angularError + 180_deg);
                }
                if(okapi::abs(lateralError) < 2_in)
                    angularError = 0_deg;
                if (isSettled(lateralError, angularError, i, positions.size() - 1))
                    break;
                int lateralOutput = lateralController->getOutput(lateralError.convert(okapi::inch));
                lateralOutput = std::clamp(lateralOutput, -maxLateral, maxLateral);
                lateralOutput *= abs(cos(angularError.convert(okapi::radian)));
                int angularOutput = angularController->getOutput(angularError.convert(okapi::degree));
                angularOutput = std::clamp(angularOutput, -maxAngular, maxAngular);
                move(lateralOutput, angularOutput);
                pros::delay(10);
            }
        }
        tare();
    }

    void Drive::driver(int forward, int turn)
    {
        setBrakeMode(driverSettings->brakeMode);
        forward = (abs(forward) < driverSettings->deadZone) ? 0 : forward;
        turn = (abs(turn) < driverSettings->deadZone) ? 0 : turn;
        forward = driverSettings->stickFunction(forward);
        turn = driverSettings->stickFunction(turn);
        forward *= driverSettings->maxPower;
        turn *= driverSettings->maxPower;
        move(forward, turn);
    }

    void Drive::move(int forward, int turn)
    {
        if (!forward && !turn)
            return applyBrakes();
        left->move(forward + turn);
        right->move(forward - turn);
    }

    void Drive::tare()
    {
        left->tare_position();
        right->tare_position();
        lateralController->reset();
        angularController->reset();
        move();
    }

    void Drive::setBrakeMode(const pros::motor_brake_mode_e &brakeMode)
    {
        left->set_brake_modes(brakeMode);
        right->set_brake_modes(brakeMode);
    }

    Drive::SPDriverSettings Drive::getDriverSettings() const
    {
        return driverSettings;
    }

    void Drive::applyBrakes()
    {
        left->move_velocity(0);
        right->move_velocity(0);
    }

    bool Drive::isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime)
    {
        return (pros::millis() * okapi::millisecond - startTime) > maxTime && maxTime != 0_s;
    }

    bool Drive::isSettled(const okapi::QLength &lateralError,
                          const okapi::QAngle &angularError,
                          int currentIndex,
                          int lastIndex)
    {
        midwayLateralSettledChecker->isSettled(lateralError);
        finalLateralSettledChecker->isSettled(lateralError);
        return (currentIndex != lastIndex &&
                midwayLateralSettledChecker->isSettled()) ||
               (currentIndex == lastIndex &&
                finalLateralSettledChecker->isSettled());
    }

    Position Drive::generateWaypoint(const Position &state, const Position &endPoint)
    {
        const okapi::QLength distanceBtwn{distance(state, endPoint)};
        const okapi::QLength x{endPoint.x - distanceBtwn * 0.6 * okapi::sin(endPoint.h)};
        const okapi::QLength y{endPoint.y - distanceBtwn * 0.6 * okapi::cos(endPoint.h)};
        return {x, y, 0_deg};
    }

    /* -------------------------------------------------------------------------- */
    /*                               Drive Builder                              */
    /* -------------------------------------------------------------------------- */

    SPDrive SPDriveBuilder::build() const
    {
        return std::make_shared<Drive>(std::make_unique<pros::MotorGroup>(leftPorts),
                                       std::make_unique<pros::MotorGroup>(rightPorts),
                                       poseEstimator,
                                       driverSettings,
                                       lateralController,
                                       angularController,
                                       finalLateralSettledChecker,
                                       midwayLateralSettledChecker,
                                       angularSettledChecker);
    }

    SPDriveBuilder SPDriveBuilder::withLeftPorts(const std::vector<int8_t> &iLeftPorts)
    {
        leftPorts = iLeftPorts;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withRightPorts(const std::vector<int8_t> &iRightPorts)
    {
        rightPorts = iRightPorts;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withPoseEstimator(SPPoseEstimator iPoseEstimator)
    {
        poseEstimator = iPoseEstimator;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withStickDeadZone(int deadZone)
    {
        driverSettings->deadZone = deadZone;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withStickFunction(const std::function<int(int)> &stickFunction)
    {
        driverSettings->stickFunction = stickFunction;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withStickMax(int maxPower)
    {
        driverSettings->maxPower = maxPower;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withBrakeMode(const pros::motor_brake_mode_e &iBrakeMode)
    {
        driverSettings->brakeMode = iBrakeMode;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withLateralController(SPController iLateralController)
    {
        lateralController = iLateralController;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withAngularController(SPController iAngularController)
    {
        angularController = iAngularController;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withFinalLateralSettledChecker(const okapi::QLength &distance,
                                                                  const okapi::QSpeed &speed,
                                                                  const okapi::QTime &time)
    {
        finalLateralSettledChecker = std::make_shared<SettledChecker<okapi::QLength, okapi::QSpeed>>(distance, speed, time);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withMidwayLateralSettledChecker(const okapi::QLength &distance)
    {
        midwayLateralSettledChecker = std::make_shared<SettledChecker<okapi::QLength, okapi::QSpeed>>(distance, 0_inps, 0_s);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withAngularSettledChecker(const okapi::QAngle &angle,
                                                             const okapi::QAngularSpeed &angularSpeed,
                                                             const okapi::QTime &time)
    {
        angularSettledChecker = std::make_shared<SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(angle, angularSpeed, time);
        return *this;
    }

}