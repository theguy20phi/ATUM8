#include "flywheel.hpp"

namespace atum8
{
    Flywheel::Flywheel(UPMotorGroup iMotorGroup,
                       SPController iVelocityController,
                       SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                       double iSpeedMultiplier) : motorGroup{std::move(iMotorGroup)},
                                                  velocityController{iVelocityController},
                                                  velocitySettledChecker{iVelocitySettledChecker},
                                                  speedMultiplier{iSpeedMultiplier}
    {
        motorGroup->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    }

    void Flywheel::taskFn()
    {
        
        motorGroup->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        while (true)
        {
            const okapi::QAngularSpeed error{referenceSpeed - getSpeed()};
            velocitySettledChecker->isSettled(error);
            double output{velocityController->getOutput(error.convert(okapi::rpm))};
            if (referenceSpeed == 0_rpm || output <= 0)
                output = 0;
            std::cout << output << " " << getSpeed().convert(okapi::rpm) << " " << error.convert(okapi::rpm) << std::endl;
            motorGroup->move(output);
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
        double avgRawVelocity{0.0};
        for (double velocity : motorGroup->get_actual_velocities())
            avgRawVelocity += velocity;
        avgRawVelocity /= motorGroup->size();
        return avgRawVelocity * speedMultiplier * okapi::rpm;
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
                                          speedMultiplier);
    }

    SPFlywheelBuilder SPFlywheelBuilder::withMotors(const std::vector<std::int8_t> &iPorts)
    {
        ports = iPorts;
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