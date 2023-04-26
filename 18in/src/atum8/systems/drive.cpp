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
                 UPVision iVision,
                 SPDriverSettings iDriverSettings,
                 SPAutonSelector iAutonSelector,
                 SPController iLateralController,
                 SPController iAngularController,
                 SPController iAimController,
                 SPLateralSettledChecker iLateralSettledChecker,
                 SPAngularSettledChecker iAngularSettledChecker,
                 SPFilter iAimFilter) : left{std::move(iLeft)},
                                        right{std::move(iRight)},
                                        poseEstimator{iPoseEstimator},
                                        vision{std::move(iVision)},
                                        driverSettings{iDriverSettings},
                                        autonSelector{iAutonSelector},
                                        lateralController{iLateralController},
                                        angularController{iAngularController},
                                        aimController{iAimController},
                                        lateralSettledChecker{iLateralSettledChecker},
                                        angularSettledChecker{iAngularSettledChecker},
                                        aimFilter{iAimFilter}
    {
        redSig = pros::Vision::signature_from_utility(1, 11497, 14799, 13148, -1561, -511, -1036, 3.000, 0);
        blueSig = pros::Vision::signature_from_utility(2, -3351, -2495, -2922, 10381, 12573, 11478, 3.000, 0);
        vision->set_signature(1, &redSig);
        vision->set_signature(2, &blueSig);
        vision->set_exposure(75);
    }

    void Drive::control(pros::Controller master)
    {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            if (driverSettings->brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST)
                driverSettings->brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
            else
                driverSettings->brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            if (driverSettings->maxPower == 1.0)
                driverSettings->maxPower = 0.5;
            else
                driverSettings->maxPower = 1.0;
        }
        double aimAssist{0.0};
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            aimAssist = visionAim();
        const int forward{master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
        const int turn{master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};
        driverMove(forward, turn + aimAssist);
    }

    void Drive::moveTo(Position target,
                       const okapi::QTime &maxTime,
                       bool reversed,
                       int maxLateral,
                       int maxAngular,
                       const okapi::QLength &offset)
    {
        tare();
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        pointAt(target, maxTime, reversed, false, maxAngular);
        target = accountForSide(target, autonSelector->getColor());
        Position state{poseEstimator->getPosition()};
        if (reversed)
            state.h += 180_deg;
        const Position initial{state.x, state.y};
        okapi::QLength lateralError{distance(initial, target) - distance(initial, state) - offset};
        okapi::QAngle angularError{okapi::OdomMath::constrainAngle180(angle(initial, target) - state.h)};
        while (!isTimeExpired(startTime, maxTime) &&
               !lateralSettledChecker->isSettled(lateralError))
        {
            state = poseEstimator->getPosition();
            if (reversed)
                state.h += 180_deg;
            lateralError = distance(initial, target) - distance(initial, state) - offset;
            angularError = okapi::OdomMath::constrainAngle180(angle(initial, target) - state.h);
            int lateralOutput{(int)lateralController->getOutput(lateralError.convert(okapi::inch))};
            int angularOutput{(int)angularController->getOutput(angularError.convert(okapi::degree))};
            if (reversed)
                lateralOutput *= -1;
            lateralOutput = std::clamp(lateralOutput, -maxLateral, maxLateral);
            angularOutput = std::clamp(angularOutput, -maxAngular, maxAngular);
            move(lateralOutput, angularOutput);
            pros::delay(stdDelay);
        }
        tare();
    }

    void Drive::pointAt(Position target,
                        const okapi::QTime &maxTime,
                        bool reversed,
                        bool useVision,
                        int maxAngular)
    {
        tare();
        target = accountForSide(target, autonSelector->getColor());
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        Position state{poseEstimator->getPosition()};
        if (reversed)
            state.h += 180_deg;
        okapi::QAngle angularError{angle(state, target)};
        while (!isTimeExpired(startTime, maxTime) &&
               !angularSettledChecker->isSettled(angularError))
        {
            state = poseEstimator->getPosition();
            if (reversed)
                state.h += 180_deg;
            angularError = angle(state, target);
            const int angularOutput{(int)angularController->getOutput(angularError.convert(okapi::degree))};
            const int visionOutput{useVision ? (int)visionAim() : 0};
            const int output{std::clamp(angularOutput + visionOutput, -maxAngular, maxAngular)};
            move(0, output);
            pros::delay(stdDelay);
        }
        tare();
    }

    double Drive::visionAim()
    {
        if (vision->get_object_count() == 0)
        {
            aimController->reset();
            return 0.0;
        }
        const double error{getVisionAimError()};
        return aimController->getOutput(error);
    }

    void Drive::driverMove(int forward, int turn)
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
        aimController->reset();
        move();
    }

    void Drive::setColor(const Color &iColor)
    {
        color = iColor;
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

    bool Drive::isSettled(const okapi::QLength &lateralError)
    {
        lateralSettledChecker->isSettled(lateralError);
        return lateralSettledChecker->isSettled();
    }

    double Drive::getVisionAimError()
    {
        auto goal = vision->get_by_sig(0, autonSelector->getColor() == Color::Red ? redSig.id : blueSig.id);
        if (autonSelector->getMatchInfo().routine == Routine::Skills)
        {
            const auto redGoal = vision->get_by_sig(0, redSig.id);
            const auto blueGoal = vision->get_by_sig(0, blueSig.id);
            const int redGoalArea{redGoal.width * redGoal.height};
            const int blueGoalArea{blueGoal.width * blueGoal.height};
            goal = redGoalArea > blueGoalArea ? redGoal : blueGoal;
        }
        return aimFilter->get((double)goal.x_middle_coord);
    }

    /* -------------------------------------------------------------------------- */
    /*                               Drive Builder                                */
    /* -------------------------------------------------------------------------- */

    SPDrive SPDriveBuilder::build() const
    {
        return std::make_shared<Drive>(std::make_unique<pros::MotorGroup>(leftPorts),
                                       std::make_unique<pros::MotorGroup>(rightPorts),
                                       poseEstimator,
                                       std::make_unique<pros::Vision>(visionPort, pros::E_VISION_ZERO_CENTER),
                                       driverSettings,
                                       autonSelector,
                                       lateralController,
                                       angularController,
                                       aimController,
                                       lateralSettledChecker,
                                       angularSettledChecker,
                                       aimFilter);
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

    SPDriveBuilder SPDriveBuilder::withVision(int8_t port)
    {
        visionPort = port;
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

    SPDriveBuilder SPDriveBuilder::withAutonSelector(SPAutonSelector iAutonSelector)
    {
        autonSelector = iAutonSelector;
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

    SPDriveBuilder SPDriveBuilder::withAimController(SPController iAimController)
    {
        aimController = iAimController;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withLateralSettledChecker(const okapi::QLength &distance,
                                                             const okapi::QSpeed &speed,
                                                             const okapi::QTime &time)
    {
        lateralSettledChecker = std::make_shared<SettledChecker<okapi::QLength, okapi::QSpeed>>(distance, speed, time);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withAngularSettledChecker(const okapi::QAngle &angle,
                                                             const okapi::QAngularSpeed &angularSpeed,
                                                             const okapi::QTime &time)
    {
        angularSettledChecker = std::make_shared<SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(angle, angularSpeed, time);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withAimFilter(SPFilter iAimFilter)
    {
        aimFilter = iAimFilter;
        return *this;
    }
}