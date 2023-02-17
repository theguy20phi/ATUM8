#pragma once

#include "atum8/constants.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/settledChecker.hpp"
#include "atum8/task.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"

#include <iostream>

namespace atum8
{
    class Flywheel : public Task
    {
    public:
        Flywheel(UPMotor iMotorA,
                 UPMotor iMotorB,
                 SPController iVelocityController,
                 SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                 double iSpeedMultiplier = 7.0);

        void taskFn();

        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        okapi::QAngularSpeed getReferenceSpeed() const;

        okapi::QAngularSpeed getSpeed() const;

        bool readyToFire(okapi::QAngularSpeed speed);

        bool readyToFire() const;

        SPController getVelocityController() const;

        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> getVelocitySettledChecker() const;

    private:
        UPMotor motorA;
        UPMotor motorB;
        double speedMultiplier{7.0};
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

        SPFlywheelBuilder withMotorA(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        SPFlywheelBuilder withMotorB(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);

        SPFlywheelBuilder withSpeedMultiplier(double iSpeedMultiplier);

        SPFlywheelBuilder withController(SPController iVelocityController);

        SPFlywheelBuilder withSettledChecker(SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker);

    private:
        int portA;
        pros::motor_gearset_e_t gearsetA;
        int portB;
        pros::motor_gearset_e_t gearsetB;
        double speedMultiplier{21.0};
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
    };
}