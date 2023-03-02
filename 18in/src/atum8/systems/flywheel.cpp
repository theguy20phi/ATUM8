#include "flywheel.hpp"

namespace atum8
{
    Flywheel::Flywheel(UPMotorGroup iMotorGroup,
                       SPController iVelocityController,
                       SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                       SPRollingAverage iRollingAverage,
                       SPSlewRate iSlewRate,
                       double iSpeedMultiplier) : motorGroup{std::move(iMotorGroup)},
                                                  velocityController{iVelocityController},
                                                  velocitySettledChecker{iVelocitySettledChecker},
                                                  rollingAverage{iRollingAverage},
                                                  slewRate{iSlewRate},
                                                  speedMultiplier{iSpeedMultiplier}
    {
        motorGroup->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    }

    void Flywheel::taskFn()
    {
        while (true)
        {
            const okapi::QAngularSpeed speed{getSpeed()};
            velocitySettledChecker->isSettled(speed, referenceSpeed);
            double output{velocityController->getOutput(speed.convert(okapi::rpm), referenceSpeed.convert(okapi::rpm))};
            if (referenceSpeed == 0_rpm || output <= 0)
                output = 0;
            if (slewRate)
                output = slewRate->slew(output);
            motorGroup->move_voltage(output);
            pros::delay(stdDelay);
        }
    }

    void Flywheel::setReferenceSpeed(const okapi::QAngularSpeed speed)
    {
        referenceSpeed = speed;
    }

    okapi::QAngularSpeed Flywheel::getReferenceSpeed() const
    {
        return referenceSpeed;
    }

    okapi::QAngularSpeed Flywheel::getSpeed() const
    {
        double avgRawVelocity{0.0};
        for (double velocity : motorGroup->get_actual_velocities())
            avgRawVelocity += velocity;
        avgRawVelocity /= motorGroup->size();
        // speedMultiplier adjusts for gearing
        avgRawVelocity *= speedMultiplier;
        if (rollingAverage)
            return rollingAverage->getAverage(avgRawVelocity) * okapi::rpm;
        return avgRawVelocity * okapi::rpm;
    }

    bool Flywheel::readyToFire(okapi::QAngularSpeed speedError)
    {
        return velocitySettledChecker->isSettled(speedError);
    }

    bool Flywheel::readyToFire() const
    {
        return velocitySettledChecker->isSettled();
    }

    SPController Flywheel::getVelocityController() const
    {
        return velocityController;
    }

    SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> Flywheel::getVelocitySettledChecker() const
    {
        return velocitySettledChecker;
    }

    /* -------------------------------------------------------------------------- */
    /*                              Flywheel Builder                              */
    /* -------------------------------------------------------------------------- */

    SPFlywheel SPFlywheelBuilder::build() const
    {
        return std::make_shared<Flywheel>(std::make_unique<pros::MotorGroup>(ports),
                                          velocityController,
                                          velocitySettledChecker,
                                          rollingAverage,
                                          slewRate,
                                          speedMultiplier);
    }

    SPFlywheelBuilder SPFlywheelBuilder::withMotors(const std::vector<std::int8_t> &iPorts)
    {
        ports = iPorts;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withSpeedMultiplier(double iSpeedMultiplier)
    {
        speedMultiplier = iSpeedMultiplier;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withController(SPController iVelocityController)
    {
        velocityController = iVelocityController;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withSettledChecker(const okapi::QAngularSpeed &maxSpeedError,
                                                            const okapi::QAngularAcceleration &maxAccelError,
                                                            const okapi::QTime &minTime)
    {
        velocitySettledChecker = std::make_shared<SettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>>(maxSpeedError,
                                                                                                                     maxAccelError,
                                                                                                                     minTime);
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withRollingAverage(int size)
    {
        rollingAverage = std::make_shared<RollingAverage>(size);
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withSlew(double maxNegChange, double maxPosChange)
    {
        slewRate = std::make_shared<SlewRate>(maxNegChange, maxPosChange);
        return *this;
    }
}