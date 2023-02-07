#pragma once

#include "constants.hpp"
#include "controllers/controller.hpp"
#include "settledChecker.hpp"
#include "task.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"

using namespace okapi::literals;

namespace atum8
{
    class Flywheel : public Task
    {
    public:
        Flywheel(UPMotor iMotor,
                 SPController iVelocityController,
                 SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker);

        void taskFn();

        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        okapi::QAngularSpeed getReferenceSpeed() const;

        okapi::QAngularSpeed getSpeed() const;

        bool readyToFire(okapi::QAngularSpeed speed);

        bool readyToFire() const;

    private:
        UPMotor motor;
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        okapi::QAngularSpeed referenceSpeed{0_degps};
    };

    using UPFlywheel = std::unique_ptr<Flywheel>;
    using SPFlywheel = std::shared_ptr<Flywheel>;

    class SPFlywheelBuilder
    {
    public:
        SPFlywheel build() const;

        SPFlywheelBuilder withMotor(int iPort,
                                     bool iReverse,
                                     const pros::motor_gearset_e_t iGearset);

        SPFlywheelBuilder withController(SPController iVelocityController);

        SPFlywheelBuilder withController(SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker);

    private:
        int port;
        bool reverse;
        pros::motor_gearset_e_t gearset;
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
    };
}