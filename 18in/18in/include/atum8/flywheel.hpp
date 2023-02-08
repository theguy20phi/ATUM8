#pragma once

#include "constants.hpp"
#include "controllers/controller.hpp"
#include "settledChecker.hpp"
#include "task.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"

namespace atum8
{
    class Flywheel : public Task
    {
    public:
        Flywheel(UPMotor iMotor,
                 SPController iVelocityController,
                 SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                 double iSpeedMultiplier = 6.0);

        void taskFn();

        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        okapi::QAngularSpeed getReferenceSpeed() const;

        okapi::QAngularSpeed getSpeed() const;

        bool readyToFire(okapi::QAngularSpeed speed);

        bool readyToFire() const;

        SPController getVelocityController() const;

        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> getVelocitySettledChecker() const;

    private:
        UPMotor motor;
        double speedMultiplier{6.0};
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
                                    bool iReverse = false,
                                    const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        SPFlywheelBuilder withSpeedMultiplier(double iSpeedMultiplier);

        SPFlywheelBuilder withController(SPController iVelocityController);

        SPFlywheelBuilder withSettledChecker(SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker);

    private:
        int port;
        bool reverse;
        pros::motor_gearset_e_t gearset;
        double speedMultiplier{6.0};
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
    };
}