#include "drive.hpp"

using namespace okapi::literals;

namespace atum8
{
    /* -------------------------------------------------------------------------- */
    /*                                   Drive                                  */
    /* -------------------------------------------------------------------------- */

    Drive::Drive(UPMotorGroup iLeft,
                 UPMotorGroup iRight,
                 double iGearing,
                 SPDimensions iDimensions,
                 SPDriverSettings iDriverSettings,
                 SPController iForwardController,
                 SPController iTurnController,
                 SPSettledChecker<okapi::QLength, okapi::QSpeed> iForwardSettledChecker,
                 SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                 SPSlewRate iForwardSlewRate,
                 SPSlewRate iTurnSlewRate,
                 UPImus iImus,
                 double iImuTrust) : left{std::move(iLeft)},
                                     right{std::move(iRight)},
                                     gearing{std::move(iGearing)},
                                     dimensions{iDimensions},
                                     driverSettings{iDriverSettings},
                                     forwardController{iForwardController},
                                     turnController{iTurnController},
                                     forwardSettledChecker{iForwardSettledChecker},
                                     turnSettledChecker{iTurnSettledChecker},
                                     forwardSlewRate{iForwardSlewRate},
                                     turnSlewRate{iTurnSlewRate},
                                     imus{std::move(iImus)},
                                     imuTrust{std::abs(iImuTrust)}
    {
    }

    void Drive::driver(int forward, int turn)
    {
        setBrakeMode(driverSettings->brakeMode);
        forward = (abs(forward) < driverSettings->deadZone) ? 0 : forward;
        turn = (abs(turn) < driverSettings->deadZone) ? 0 : turn;
        forward = driverSettings->stickFunction(forward);
        turn = driverSettings->stickFunction(turn);
        forward = driverSettings->forwardSlewRate ? driverSettings->forwardSlewRate->slew(forward) : forward;
        turn = driverSettings->turnSlewRate ? driverSettings->turnSlewRate->slew(turn) : turn;
        forward *= driverSettings->maxPower;
        turn *= driverSettings->maxPower;
        move(forward, turn);
    }

    void Drive::forward(const okapi::QLength &distance, const okapi::QTime &maxTime, int maxForward)
    {
        toReference([this, distance]()
                    { return distance - getDistance(); },
                    [this]()
                    { return okapi::OdomMath::constrainAngle180(-getAngle()); }, // Deviation from initial angle
                    maxTime,
                    maxForward);
    }

    void Drive::turn(const okapi::QAngle &angle, const okapi::QTime &maxTime, int maxTurn)
    {
        toReference([this]()
                    { return 0_m; }, // Shouldn't use forward controller
                    [this, angle]()
                    { return okapi::OdomMath::constrainAngle180(angle - getAngle()); },
                    maxTime,
                    0,
                    maxTurn);
    }

    void Drive::move(int forward, int turn)
    {
        if (!forward && !turn)
            return applyBrakes();
        left->move(forward + turn);
        right->move(forward - turn);
    }

    okapi::QLength Drive::getDistance() const
    {
        std::vector<double> positions = left->get_positions();
        std::vector<double> rightPositions = right->get_positions();
        positions.insert(positions.end(), rightPositions.begin(), rightPositions.end());
        double avgRotation{std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size()};
        return avgRotation * gearing / 360 * dimensions->wheelCircum;
    }

    okapi::QAngle Drive::getAngle() const
    {
        std::vector<double> leftPositions = left->get_positions();
        std::vector<double> rightPositions = right->get_positions();
        const double lAvgRotation{std::accumulate(leftPositions.begin(), leftPositions.end(), 0.0) / leftPositions.size()};
        const double rAvgRotation{std::accumulate(rightPositions.begin(), rightPositions.end(), 0.0) / rightPositions.size()};
        const okapi::QLength diff{(lAvgRotation - rAvgRotation) / 360 * dimensions->wheelCircum};
        const okapi::QAngle driveAngle{(gearing * diff / dimensions->baseWidth) * okapi::radian};
        if (imus)
            return imus->get_rotation() * imuTrust * okapi::degree + driveAngle * (1 - imuTrust);
        return driveAngle;
    }

    bool Drive::isSettled(const okapi::QLength &distanceError, const okapi::QAngle &angleError)
    {
        return forwardSettledChecker->isSettled(distanceError) &&
               turnSettledChecker->isSettled(angleError);
    }

    void Drive::reset()
    {
        move();
        tare();
        // This will calibrate the IMU and block!
        if (imus)
            imus->reset();
    }

    void Drive::tare()
    {
        left->tare_position();
        right->tare_position();
        imus->tare_rotation();
        forwardController->reset();
        turnController->reset();
        forwardSlewRate->reset();
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

    bool Drive::isTimeNotExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime)
    {
        return (pros::millis() * okapi::millisecond - startTime) <= maxTime || maxTime == 0_s;
    }

    void Drive::toReference(const std::function<okapi::QLength()> &distanceError,
                            const std::function<okapi::QAngle()> &angleError,
                            const okapi::QTime &maxTime,
                            int maxForward,
                            int maxTurn)
    {
        tare();
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (!isSettled(distanceError(), angleError()) &&
               isTimeNotExpired(startTime, maxTime))
        {
            move(useForwardController(distanceError(), maxForward),
                 useTurnController(angleError(), maxTurn));
            pros::delay(atum8::stdDelay);
        }
        move(); // Stop the drive
    }

    int Drive::useForwardController(const okapi::QLength &distanceError, int maxForward)
    {
        double forwardOutput{forwardController->getOutput(distanceError.convert(okapi::inch))};
        forwardOutput = forwardSlewRate ? forwardSlewRate->slew(forwardOutput) : forwardOutput;
        return std::clamp((int)forwardOutput, -maxForward, maxForward);
    }

    int Drive::useTurnController(const okapi::QAngle &angleError, int maxTurn)
    {
        double turnOutput{turnController->getOutput(angleError.convert(okapi::degree))};
        turnOutput = turnSlewRate ? turnSlewRate->slew(turnOutput) : turnOutput;
        return std::clamp((int)turnOutput, -maxTurn, maxTurn);
    }

    /* -------------------------------------------------------------------------- */
    /*                               Drive Builder                              */
    /* -------------------------------------------------------------------------- */

    SPDrive SPDriveBuilder::build() const
    {
        return std::make_shared<Drive>(std::make_unique<pros::MotorGroup>(leftPorts),
                                       std::make_unique<pros::MotorGroup>(rightPorts),
                                       gearing,
                                       dimensions,
                                       driverSettings,
                                       forwardController, turnController,
                                       forwardSettledChecker, turnSettledChecker,
                                       forwardSlewRate, turnSlewRate,
                                       std::make_unique<Imus>(imuPorts), imuTrust);
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

    SPDriveBuilder SPDriveBuilder::withGearing(double iGearing) 
    {
        gearing = iGearing;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withBaseWidth(const okapi::QLength &baseWidth)
    {
        dimensions->baseWidth = baseWidth;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withWheelCircum(const okapi::QLength &wheelCircum)
    {
        dimensions->wheelCircum = wheelCircum;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withStickDeadZone(int deadZone)
    {
        driverSettings->deadZone = deadZone;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::witStickSlew(double slew)
    {
        driverSettings->forwardSlewRate = std::make_shared<SlewRate>(slew);
        driverSettings->strafeSlewRate = std::make_shared<SlewRate>(slew);
        driverSettings->turnSlewRate = std::make_shared<SlewRate>(slew);
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

    SPDriveBuilder SPDriveBuilder::withForwardController(SPController iForwardController)
    {
        forwardController = iForwardController;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withTurnController(SPController iTurnController)
    {
        turnController = iTurnController;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withForwardSettledChecker(const okapi::QLength &distance,
                                                             const okapi::QSpeed &speed,
                                                             const okapi::QTime &time)
    {
        forwardSettledChecker = std::make_shared<SettledChecker<okapi::QLength, okapi::QSpeed>>(distance, speed, time);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withTurnSettledChecker(const okapi::QAngle &angle,
                                                          const okapi::QAngularSpeed &angularSpeed,
                                                          const okapi::QTime &time)
    {
        turnSettledChecker = std::make_shared<SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(angle, angularSpeed, time);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withForwardSlew(double slewRate)
    {
        forwardSlewRate = std::make_shared<SlewRate>(slewRate);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withTurnSlew(double slewRate)
    {
        turnSlewRate = std::make_shared<SlewRate>(slewRate);
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withBrakeMode(const pros::motor_brake_mode_e &iBrakeMode)
    {
        driverSettings->brakeMode = iBrakeMode;
        return *this;
    }

    SPDriveBuilder SPDriveBuilder::withImus(const std::vector<int> &ports, double trust)
    {
        imuPorts = ports;
        imuTrust = trust;
        return *this;
    }
}