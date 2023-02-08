#include "mecanum.hpp"

using namespace okapi::literals;

namespace atum8
{
    /* -------------------------------------------------------------------------- */
    /*                                   Mecanum                                  */
    /* -------------------------------------------------------------------------- */

    Mecanum::Mecanum(UPMotor iRFMotor,
                     UPMotor iLFMotor,
                     UPMotor iLBMotor,
                     UPMotor iRBMotor,
                     SPDimensions iDimensions,
                     SPDriverSettings iDriverSettings,
                     SPController iForwardController,
                     SPController iTurnController,
                     SPSettledChecker<okapi::QLength, okapi::QSpeed> iForwardSettledChecker,
                     SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                     SPSlewRate iForwardSlewRate,
                     SPSlewRate iTurnSlewRate,
                     const pros::motor_brake_mode_e &brakeMode,
                     UPImu iImu,
                     double iImuTrust) : rFMotor{std::move(iRFMotor)},
                                         lFMotor{std::move(iLFMotor)},
                                         lBMotor{std::move(iLBMotor)},
                                         rBMotor{std::move(iRBMotor)},
                                         dimensions{iDimensions},
                                         driverSettings{iDriverSettings},
                                         forwardController{iForwardController},
                                         turnController{iTurnController},
                                         forwardSettledChecker{iForwardSettledChecker},
                                         turnSettledChecker{iTurnSettledChecker},
                                         forwardSlewRate{iForwardSlewRate},
                                         turnSlewRate{iTurnSlewRate},
                                         imu{std::move(iImu)},
                                         imuTrust{std::abs(iImuTrust)}
    {
        setBrakeMode(brakeMode);
    }

    void Mecanum::driver(int forward, int strafe, int turn)
    {
        forward = (abs(forward) < driverSettings->deadZone) ? 0 : forward;
        strafe = (abs(strafe) < driverSettings->deadZone) ? 0 : strafe;
        turn = (abs(turn) < driverSettings->deadZone) ? 0 : turn;
        forward = driverSettings->stickFunction(forward);
        strafe = driverSettings->stickFunction(strafe);
        turn = driverSettings->stickFunction(turn);
        forward = std::clamp(forward, -driverSettings->maxPower, driverSettings->maxPower);
        strafe = std::clamp(strafe, -driverSettings->maxPower, driverSettings->maxPower);
        turn = std::clamp(turn, -driverSettings->maxPower, driverSettings->maxPower);
        forward = driverSettings->forwardSlewRate ? driverSettings->forwardSlewRate->slew(forward) : forward;
        strafe = driverSettings->strafeSlewRate ? driverSettings->strafeSlewRate->slew(strafe) : strafe;
        turn = driverSettings->turnSlewRate ? driverSettings->turnSlewRate->slew(turn) : turn;
        move(forward, strafe, turn);
    }

    void Mecanum::forward(const okapi::QLength &distance, const okapi::QTime &maxTime, int maxForward)
    {
        tare();
        setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        okapi::QLength distanceError{distance - getDistance()};
        okapi::QAngle angleError{getAngle()};
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (!isSettled(distanceError, angleError) &&
               !isTimeExpired(startTime, maxTime))
        {
            distanceError = distance - getDistance();
            angleError = getAngle();
            move(useForwardController(distanceError, maxForward),
                 useTurnController(angleError));
        }
        move(); // Stop the drive
    }

    void Mecanum::turn(const okapi::QAngle &angle, const okapi::QTime &maxTime, int maxTurn)
    {
        tare();
        setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        okapi::QAngle angleError{angle - getAngle()};
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (!turnSettledChecker->isSettled(angleError) &&
               !isTimeExpired(startTime, maxTime))
        {
            angleError = angle - getAngle();
            move(0, useTurnController(angleError, maxTurn));
        }
        move(); // Stop the drive
    }

    void Mecanum::move(int forward, int strafe, int turn)
    {
        if (forward || strafe || turn)
        {
            rFMotor->move(forward - strafe - turn);
            lFMotor->move(forward + strafe + turn);
            lBMotor->move(forward - strafe + turn);
            rBMotor->move(forward + strafe - turn);
        }
        else
            applyBrakes();
    }

    okapi::QLength Mecanum::getDistance() const
    {
        double avgRotation{0};
        avgRotation += rFMotor->get_position();
        avgRotation += lFMotor->get_position();
        avgRotation += lBMotor->get_position();
        avgRotation += rBMotor->get_position();
        avgRotation /= 4;
        return avgRotation * dimensions->wheelCircum;
    }

    okapi::QAngle Mecanum::getAngle() const
    {
        const double lAvgRotation{(lFMotor->get_position() + lBMotor->get_position()) / 2};
        const double rAvgRotation{(rFMotor->get_position() + rBMotor->get_position()) / 2};
        const okapi::QLength diff{(lAvgRotation - rAvgRotation) * dimensions->wheelCircum};
        const okapi::QAngle driveAngle{(diff / dimensions->baseWidth) * okapi::radian};
        if (imu)
            return imu->get_rotation() * imuTrust * okapi::degree + driveAngle * (1 - imuTrust);
        return driveAngle;
    }

    bool Mecanum::isSettled(const okapi::QLength &distanceError, const okapi::QAngle &angleError)
    {
        return forwardSettledChecker->isSettled(distanceError) &&
               turnSettledChecker->isSettled(angleError);
    }

    void Mecanum::reset()
    {
        move();
        tare();
        forwardController->reset();
        turnController->reset();
        // This will calibrate the IMU and block!
        imu->reset(true);
    }

    void Mecanum::tare()
    {
        rFMotor->tare_position();
        lFMotor->tare_position();
        lBMotor->tare_position();
        rBMotor->tare_position();
        imu->tare_rotation();
    }

    void Mecanum::setBrakeMode(const pros::motor_brake_mode_e &brakeMode)
    {
        rFMotor->set_brake_mode(brakeMode);
        lFMotor->set_brake_mode(brakeMode);
        lBMotor->set_brake_mode(brakeMode);
        rBMotor->set_brake_mode(brakeMode);
    }

    Mecanum::SPDimensions Mecanum::getDimensions() const
    {
        return dimensions;
    }

    Mecanum::SPDriverSettings Mecanum::getDriverSettings() const
    {
        return driverSettings;
    }

    SPController Mecanum::getForwardController() const
    {
        return forwardController;
    }

    SPController Mecanum::getTurnController() const
    {
        return turnController;
    }

    SPSettledChecker<okapi::QLength, okapi::QSpeed> Mecanum::getForwardSettledChecker() const
    {
        return forwardSettledChecker;
    }

    SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> Mecanum::getTurnSettledChecker() const
    {
        return turnSettledChecker;
    }

    SPSlewRate Mecanum::getForwardSlewRate() const
    {
        return forwardSlewRate;
    }

    SPSlewRate Mecanum::getTurnSlewRate() const
    {
        return turnSlewRate;
    }

    void Mecanum::applyBrakes()
    {
        rFMotor->move_velocity(0);
        lFMotor->move_velocity(0);
        lBMotor->move_velocity(0);
        rBMotor->move_velocity(0);
    }

    bool Mecanum::isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime)
    {
        return (pros::millis() * okapi::millisecond - startTime) <= maxTime;
    }

    int Mecanum::useForwardController(const okapi::QLength &distanceError, int maxForward)
    {
        double forwardOutput{forwardController->getOutput(distanceError.convert(okapi::inch))};
        forwardOutput = forwardSlewRate ? forwardSlewRate->slew(forwardOutput) : forwardOutput;
        return std::clamp((int)forwardOutput, -maxForward, maxForward);
    }

    int Mecanum::useTurnController(const okapi::QAngle &angleError, int maxTurn)
    {
        double turnOutput{turnController->getOutput(angleError.convert(okapi::degree))};
        turnOutput = turnSlewRate ? turnSlewRate->slew(turnOutput) : turnOutput;
        return std::clamp((int)turnOutput, -maxTurn, maxTurn);
    }

    /* -------------------------------------------------------------------------- */
    /*                               Mecanum Builder                              */
    /* -------------------------------------------------------------------------- */

    SPMecanum SPMecanumBuilder::build() const
    {
        return std::make_shared<Mecanum>(std::make_unique<pros::Motor>(rFPort, rFGearset, rFReverse),
                                         std::make_unique<pros::Motor>(lFPort, lFGearset, lFReverse),
                                         std::make_unique<pros::Motor>(lBPort, lBGearset, lBReverse),
                                         std::make_unique<pros::Motor>(rBPort, rBGearset, rBReverse),
                                         dimensions,
                                         driverSettings,
                                         forwardController, turnController,
                                         forwardSettledChecker, turnSettledChecker,
                                         forwardSlewRate, turnSlewRate,
                                         brakeMode,
                                         std::make_unique<pros::Imu>(imuPort), imuTrust);
    }

    SPMecanumBuilder SPMecanumBuilder::withRFMotor(int port,
                                                   bool reverse,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        rFPort = port;
        rFReverse = reverse;
        rFGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withLFMotor(int port,
                                                   bool reverse,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        lFPort = port;
        lFReverse = reverse;
        lFGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withLBMotor(int port,
                                                   bool reverse,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        lBPort = port;
        lBReverse = reverse;
        lBGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withRBMotor(int port,
                                                   bool reverse,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        rBPort = port;
        rBReverse = reverse;
        rBGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withBaseWidth(const okapi::QLength &baseWidth)
    {
        dimensions->baseWidth = baseWidth;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withWheelCircum(const okapi::QLength &wheelCircum)
    {
        dimensions->wheelCircum = wheelCircum;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withStickDeadZone(int deadZone)
    {
        driverSettings->deadZone = deadZone;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::witStickSlew(double slew)
    {
        driverSettings->forwardSlewRate = std::make_shared<SlewRate>(slew);
        driverSettings->strafeSlewRate = std::make_shared<SlewRate>(slew);
        driverSettings->turnSlewRate = std::make_shared<SlewRate>(slew);
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withStickFunction(const std::function<int(int)> &stickFunction)
    {
        driverSettings->stickFunction = stickFunction;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withStickMax(int maxPower)
    {
        driverSettings->maxPower = maxPower;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withForwardController(SPController iForwardController)
    {
        forwardController = iForwardController;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withTurnController(SPController iTurnController)
    {
        turnController = iTurnController;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withForwardSettledChecker(const okapi::QLength &distance,
                                                                 const okapi::QSpeed &speed,
                                                                 const okapi::QTime &time)
    {
        forwardSettledChecker = std::make_shared<SettledChecker<okapi::QLength, okapi::QSpeed>>(distance, speed, time);
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withTurnSettledChecker(const okapi::QAngle &angle,
                                                              const okapi::QAngularSpeed &angularSpeed,
                                                              const okapi::QTime &time)
    {
        turnSettledChecker = std::make_shared<SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(angle, angularSpeed, time);
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withForwardSlew(double slewRate)
    {
        forwardSlewRate = std::make_shared<SlewRate>(slewRate);
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withTurnSlew(double slewRate)
    {
        turnSlewRate = std::make_shared<SlewRate>(slewRate);
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withBrakeMode(const pros::motor_brake_mode_e &iBrakeMode)
    {
        brakeMode = iBrakeMode;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withImu(int port, double trust)
    {
        imuPort = port;
        imuTrust = trust;
        return *this;
    }
}