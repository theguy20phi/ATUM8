#include "flywheel.hpp"

namespace atum8
{
    Flywheel::Flywheel(UPMotor iMotor,
                       SPController iVelocityController,
                       SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                       double iSpeedMultiplier) : motor{std::move(motor)},
                                                  velocityController{iVelocityController},
                                                  velocitySettledChecker{iVelocitySettledChecker},
                                                  speedMultiplier{iSpeedMultiplier} {}

    void Flywheel::taskFn()
    {
        while (true)
        {
            const okapi::QAngularSpeed error{getSpeed() - referenceSpeed};
            velocitySettledChecker->isSettled(error);
            double output{velocityController->getOutput(error.convert(okapi::rpm))};
            if (output < 0)
                output = 0;
            motor->move(output);
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
        // speedMultiplier adjusts for modifications to motor cartridges
        return motor->get_actual_velocity() * speedMultiplier * okapi::rpm;
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
        return std::make_shared<Flywheel>(std::make_unique<pros::Motor>(port, gearset, reverse),
                                          velocityController,
                                          velocitySettledChecker);
    }

    SPFlywheelBuilder SPFlywheelBuilder::withMotor(int iPort,
                                                   bool iReverse,
                                                   const pros::motor_gearset_e_t &iGearset)
    {
        port = iPort;
        reverse = iReverse;
        gearset = iGearset;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withController(SPController iVelocityController)
    {
        velocityController = iVelocityController;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withSettledChecker(SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker)
    {
        velocitySettledChecker = iVelocitySettledChecker;
        return *this;
    }
}