#pragma once

#include "atum8/constants.hpp"
#include "atum8/task.hpp"

namespace atum8
{
    class Roller : public Task
    {
    public:
        Roller(UPMotor iMotor, UPOptical iOptical, const Color &iColor = Color::Red);

        void taskFn();

        void runRoller(int speed = 50);

        void turnToColor();

        void setColor(const Color &iColor);

        Color getColor() const;

    private:
        UPMotor motor;
        UPOptical optical;
        Color color{Color::Red};
        bool turningToColor{false};
    };

    using UPRoller = std::unique_ptr<Roller>;
    using SPRoller = std::shared_ptr<Roller>;

    class SPRollerBuilder
    {
    public:
        SPRoller build() const;

        SPRollerBuilder withMotor(int iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        SPRollerBuilder withOptical(int iPort);

        SPRollerBuilder withColor(const Color &iColor);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
        int opticalPort;
        Color color;
    };
}