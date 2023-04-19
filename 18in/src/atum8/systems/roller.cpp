#include "roller.hpp"

namespace atum8
{
    Roller::Roller(UPMotor iMotor) : motor{std::move(iMotor)}
    {
        motor->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
    }

    void Roller::control(pros::Controller master)
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            runRoller(127);
        else
            runRoller(0);
    }

    void Roller::runRoller(int speed)
    {
        motor->move_velocity(speed);
    }

    void Roller::stopRoller()
    {
        motor->move(0);
    }

    void Roller::runForAt(double position, int velocity)
    {
        motor->move_relative(position, velocity);
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