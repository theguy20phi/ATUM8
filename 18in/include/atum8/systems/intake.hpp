#pragma once

#include "atum8/constants.hpp"
#include "atum8/task.hpp"
#include "flywheel.hpp"

namespace atum8
{
    class Intake : public Task
    {
    public:
        Intake(UPMotor iMotor,
               UPADIDigitalOut iPiston,
               const SPFlywheel &iFlywheel,
               int iShotDelay = 500);

        void taskFn();

        void runIntake(int speed = 127);

        void stopIntake();

        void shoot(int iShooting = 1, bool iFlywheelBlocks = true);

        bool isShooting() const;

    private:
        bool shouldShoot();
        UPMotor motor;
        UPADIDigitalOut piston;
        SPFlywheel flywheel;
        const int shotDelay;
        int shooting{0};
        bool flywheelBlocks{true};
    };

    using UPIntake = std::unique_ptr<Intake>;
    using SPIntake = std::shared_ptr<Intake>;

    class SPIntakeBuilder
    {
    public:
        SPIntake build() const;

        SPIntakeBuilder withMotor(int iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        SPIntakeBuilder withPiston(int iPort);

        SPIntakeBuilder withFlywheel(const SPFlywheel &iFlywheel);

        SPIntakeBuilder withShotDelay(int iShotDelay = 500);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
        int pistonPort;
        SPFlywheel flywheel;
        int shotDelay;
    };
}