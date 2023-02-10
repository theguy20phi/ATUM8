#include "roller.hpp"

namespace atum8
{
    Roller::Roller(UPMotor iMotor) : motor{std::move(iMotor)}
    {
    }

    void Roller::runForAt(int encoderUnits, int speed)
    {
        motor->move_relative(encoderUnits, speed);
    }

    void Roller::runRoller(int speed)
    {
        motor->move(speed);
    }

    SPRoller SPRollerBuilder::build() const
    {
        return std::make_shared<Roller>(std::make_unique<pros::Motor>(motorPort, gearset));
    }

    SPRollerBuilder SPRollerBuilder::withMotor(int iPort,
                                               const pros::motor_gearset_e_t &iGearset)
    {
        motorPort = iPort;
        gearset = iGearset;
        return *this;
    }
}