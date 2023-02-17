#include "flywheel.hpp"

namespace atum8
{
    Flywheel::Flywheel(UPMotor iMotorA,
                       UPMotor iMotorB,
                       SPController iVelocityController,
                       SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                       double iSpeedMultiplier) : motorA{std::move(iMotorA)},
                                                  motorB{std::move(iMotorB)},
                                                  velocityController{iVelocityController},
                                                  velocitySettledChecker{iVelocitySettledChecker},
                                                  speedMultiplier{iSpeedMultiplier}
    {
        motorA->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        motorB->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    }

    void Flywheel::taskFn()
    {
        while (true)
        {
            const okapi::QAngularSpeed error{referenceSpeed - getSpeed()};
            velocitySettledChecker->isSettled(error);
            double output{velocityController->getOutput(error.convert(okapi::rpm))};
            if (referenceSpeed == 0_rpm || output <= 0)
                output = 0;
            std::cout << getSpeed().convert(okapi::rpm) << " " << error.convert(okapi::rpm) << std::endl;
            motorA->move(output);
            motorB->move(output);
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
        double velocity{motorA->get_actual_velocity() + motorB->get_actual_velocity()};
        velocity /= 2.0;
        // speedMultiplier adjusts for modifications to motor cartridges
        return velocity * speedMultiplier * okapi::rpm;
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
        return std::make_shared<Flywheel>(std::make_unique<pros::Motor>(portA, gearsetA),
                                          std::make_unique<pros::Motor>(portB, gearsetB),
                                          velocityController,
                                          velocitySettledChecker,
                                          speedMultiplier);
    }

    SPFlywheelBuilder SPFlywheelBuilder::withMotorA(int port,
                                                    const pros::motor_gearset_e_t &gearset)
    {
        portA = port;
        gearsetA = gearset;
        return *this;
    }

    SPFlywheelBuilder SPFlywheelBuilder::withMotorB(int port,
                                                    const pros::motor_gearset_e_t &gearset)
    {
        portB = port;
        gearsetB = gearset;
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

    SPFlywheelBuilder SPFlywheelBuilder::withSettledChecker(SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker)
    {
        velocitySettledChecker = iVelocitySettledChecker;
        return *this;
    }
}