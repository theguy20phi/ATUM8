#include "mecanum.hpp"

using namespace okapi::literals;

namespace atum8
{
    Mecanum::Mecanum(UPMotor iRFMotor,
                     UPMotor iLFMotor,
                     UPMotor iLBMotor,
                     UPMotor iRBMotor,
                     const Dimensions &iDimensions,
                     UPController iForwardController,
                     UPController iTurnController,
                     UPSettledChecker<okapi::QLength, okapi::QSpeed> iForwardSettledChecker,
                     UPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                     UPSlewRate iForwardSlewRate,
                     UPSlewRate iTurnSlewRate,
                     UPImu iImu,
                     double iImuTrust) : rFMotor{std::move(iRFMotor)},
                                         lFMotor{std::move(iLFMotor)},
                                         lBMotor{std::move(iLBMotor)},
                                         rBMotor{std::move(iRBMotor)},
                                         dimensions{iDimensions},
                                         forwardController{std::move(iForwardController)},
                                         turnController{std::move(iTurnController)},
                                         forwardSettledChecker{std::move(iForwardSettledChecker)},
                                         turnSettledChecker{std::move(iTurnSettledChecker)},
                                         forwardSlewRate{std::move(iForwardSlewRate)},
                                         turnSlewRate{std::move(iTurnSlewRate)},
                                         imu{std::move(iImu)},
                                         imuTrust{std::abs(iImuTrust)}
    {
    }

    void Mecanum::forward(const okapi::QLength &distance, const okapi::QTime &maxTime, int maxForward)
    {
        tare();
        okapi::QLength distanceError{distance - getDistance()};
        okapi::QAngle angleError{getAngle()};
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (!isSettled(distanceError, angleError) &&
               !isTimeExpired(startTime, maxTime))
        {
            distanceError = distance - getDistance();
            angleError = getAngle();
            move(driveForwardController(distanceError, maxForward),
                 driveTurnController(angleError));
        }
    }

    void Mecanum::turn(const okapi::QAngle &angle, const okapi::QTime &maxTime, int maxTurn)
    {
        tare();
        okapi::QAngle angleError{angle - getAngle()};
        const okapi::QTime startTime{pros::millis() * okapi::millisecond};
        while (!turnSettledChecker->isSettled(angleError) &&
               !isTimeExpired(startTime, maxTime))
        {
            angleError = angle - getAngle();
            move(0, driveTurnController(angleError, maxTurn));
        }
    }

    void Mecanum::move(int forward, int strafe, int turn)
    {
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
        return avgRotation * dimensions.wheelCircum;
    }

    okapi::QAngle Mecanum::getAngle() const
    {
        const double lAvgRotation{(lFMotor->get_position() + lBMotor->get_position()) / 2};
        const double rAvgRotation{(rFMotor->get_position() + rBMotor->get_position()) / 2};
        const okapi::QLength diff{(lAvgRotation - rAvgRotation) * dimensions.wheelCircum};
        const okapi::QAngle driveAngle{(diff / dimensions.baseWidth) * okapi::radian};
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

    bool Mecanum::isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime)
    {
        return (pros::millis() * okapi::millisecond - startTime) <= maxTime;
    }

    int Mecanum::driveForwardController(const okapi::QLength &distanceError, int maxForward)
    {
        double forwardOutput{forwardController->getOutput(distanceError.convert(okapi::inch))};
        forwardOutput = forwardSlewRate->slew(forwardOutput);
        return std::clamp((int)forwardOutput, -maxForward, maxForward);
    }

    int Mecanum::driveTurnController(const okapi::QAngle &angleError, int maxTurn)
    {
        double turnOutput{turnController->getOutput(angleError.convert(okapi::degree))};
        turnOutput = turnSlewRate->slew(turnOutput);
        return std::clamp((int)turnOutput, -maxTurn, maxTurn);
    }
}