#pragma once

#include "atum8/constants.hpp"
#include "atum8/task.hpp"

namespace atum8
{
    class Roller
    {
    public:
        Roller(UPMotor iMotor);

        void runForAt(int encoderUnits, int speed = 50);

        void runRoller(int speed = 50);

    private:
        UPMotor motor;
    };

    using UPRoller = std::unique_ptr<Roller>;
    using SPRoller = std::shared_ptr<Roller>;

    class SPRollerBuilder
    {
    public:
        SPRoller build() const;

        SPRollerBuilder withMotor(int iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
    };
}