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
                     UPImus iImus,
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
                                         imus{std::move(iImus)},
                                         imuTrust{std::abs(iImuTrust)}
    {
    }

    void Mecanum::driver(int forward, int strafe, int turn)
    {
        setBrakeMode(driverSettings->brakeMode);
        forward = (abs(forward) < driverSettings->deadZone) ? 0 : forward;
        strafe = (abs(strafe) < driverSettings->deadZone) ? 0 : strafe;
        turn = (abs(turn) < driverSettings->deadZone) ? 0 : turn;
        forward = driverSettings->stickFunction(forward);
        strafe = driverSettings->stickFunction(strafe);
        turn = driverSettings->stickFunction(turn);
        forward = driverSettings->forwardSlewRate ? driverSettings->forwardSlewRate->slew(forward) : forward;
        strafe = driverSettings->strafeSlewRate ? driverSettings->strafeSlewRate->slew(strafe) : strafe;
        turn = driverSettings->turnSlewRate ? driverSettings->turnSlewRate->slew(turn) : turn;
        forward *= driverSettings->maxPower;
        strafe *= driverSettings->maxPower;
        turn *= driverSettings->maxPower;
        move(forward, strafe, turn);
    }

    void Mecanum::forward(const okapi::QLength &distance, const okapi::QTime &maxTime, int maxForward)
    {
        toReference([this, distance]()
                    { return distance - getDistance(); },
                    [this]()
                    { return okapi::OdomMath::constrainAngle180(-getAngle()); }, // Deviation from initial angle
                    maxTime,
                    maxForward);
    }

    void Mecanum::turn(const okapi::QAngle &angle, const okapi::QTime &maxTime, int maxTurn)
    {
        toReference([this]()
                    { return 0_m; }, // Shouldn't use forward controller
                    [this, angle]()
                    { return okapi::OdomMath::constrainAngle180(angle - getAngle()); },
                    maxTime,
                    0,
                    maxTurn);
    }

    void Mecanum::move(int forward, int strafe, int turn)
    {   
        if(!forward && !strafe && !turn) 
            return applyBrakes();
        rFMotor->move(forward - strafe - turn);
        lFMotor->move(forward + strafe + turn);
        lBMotor->move(forward - strafe + turn);
        rBMotor->move(forward + strafe - turn);
    }

    okapi::QLength Mecanum::getDistance() const
    {
        double avgRotation{0};
        avgRotation += rFMotor->get_position();
        avgRotation += lFMotor->get_position();
        avgRotation += lBMotor->get_position();
        avgRotation += rBMotor->get_position();
        avgRotation /= 4;
        return avgRotation / 360 * dimensions->wheelCircum;
    }

    okapi::QAngle Mecanum::getAngle() const
    {
        const double lAvgRotation{(lFMotor->get_position() + lBMotor->get_position()) / 2};
        const double rAvgRotation{(rFMotor->get_position() + rBMotor->get_position()) / 2};
        const okapi::QLength diff{(lAvgRotation - rAvgRotation) / 360 * dimensions->wheelCircum};
        const okapi::QAngle driveAngle{(diff / dimensions->baseWidth) * okapi::radian};
        if (imus)
            return imus->get_rotation() * imuTrust * okapi::degree + driveAngle * (1 - imuTrust);
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
        // This will calibrate the IMU and block!
        if (imus)
            imus->reset();
    }

    void Mecanum::tare()
    {
        rFMotor->tare_position();
        lFMotor->tare_position();
        lBMotor->tare_position();
        rBMotor->tare_position();
        imus->tare_rotation();
        forwardController->reset();
        turnController->reset();
        forwardSlewRate->reset();
        move();
    }

    void Mecanum::setBrakeMode(const pros::motor_brake_mode_e &brakeMode)
    {
        rFMotor->set_brake_mode(brakeMode);
        lFMotor->set_brake_mode(brakeMode);
        lBMotor->set_brake_mode(brakeMode);
        rBMotor->set_brake_mode(brakeMode);
    }

    Mecanum::SPDriverSettings Mecanum::getDriverSettings() const
    {
        return driverSettings;
    }

    void Mecanum::applyBrakes() 
    {
        rFMotor->move_velocity(0);
        lFMotor->move_velocity(0);
        lBMotor->move_velocity(0);
        rBMotor->move_velocity(0);
    }

    bool Mecanum::isTimeNotExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime)
    {
        return (pros::millis() * okapi::millisecond - startTime) <= maxTime || maxTime == 0_s;
    }

    void Mecanum::toReference(const std::function<okapi::QLength()> &distanceError,
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
                 0,
                 useTurnController(angleError(), maxTurn));
            pros::delay(atum8::stdDelay);
        }
        move(); // Stop the drive
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
        return std::make_shared<Mecanum>(std::make_unique<pros::Motor>(rFPort, rFGearset),
                                         std::make_unique<pros::Motor>(lFPort, lFGearset),
                                         std::make_unique<pros::Motor>(lBPort, lBGearset),
                                         std::make_unique<pros::Motor>(rBPort, rBGearset),
                                         dimensions,
                                         driverSettings,
                                         forwardController, turnController,
                                         forwardSettledChecker, turnSettledChecker,
                                         forwardSlewRate, turnSlewRate,
                                         std::make_unique<Imus>(imuPorts), imuTrust);
    }

    SPMecanumBuilder SPMecanumBuilder::withRFMotor(int port,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        rFPort = port;
        rFGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withLFMotor(int port,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        lFPort = port;
        lFGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withLBMotor(int port,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        lBPort = port;
        lBGearset = gearset;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withRBMotor(int port,
                                                   const pros::motor_gearset_e_t &gearset)
    {
        rBPort = port;
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
        driverSettings->brakeMode = iBrakeMode;
        return *this;
    }

    SPMecanumBuilder SPMecanumBuilder::withImus(const std::vector<int> &ports, double trust)
    {
        imuPorts = ports;
        imuTrust = trust;
        return *this;
    }
}