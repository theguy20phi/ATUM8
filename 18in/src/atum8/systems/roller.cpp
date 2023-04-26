#include "roller.hpp"

namespace atum8
{
    Roller::Roller(UPMotor iMotor,
                   UPOptical iOpticalA,
                   UPOptical iOpticalB,
                   SPAutonSelector iAutonSelector,
                   int iRedHue,
                   int iBlueHue) : motor{std::move(iMotor)},
                                   opticalA{std::move(iOpticalA)},
                                   opticalB{std::move(iOpticalB)},
                                   autonSelector{iAutonSelector},
                                   redHue{iRedHue},
                                   blueHue{iBlueHue}
    {
        motor->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
        opticalA->set_led_pwm(100);
        opticalB->set_led_pwm(100);
        addTaskFns({rollerControlTask()});
    }

    TaskFn Roller::rollerControlTask()
    {
        return [=]() {
            while(true) {
                switch(rollerState) {
                    case RollerState::TurningToColor:
                        turnToColor();
                        break;
                    case RollerState::Turning:
                        runRoller();
                        break;
                    default:
                        stopRoller();
                        break;
                }
                pros::delay(stdDelay);
            }
        };
    }

    void Roller::setState(const Roller::RollerState &iRollerState)
    {
        rollerState = iRollerState;
    }

    Roller::RollerState Roller::getState() const 
    {
        return rollerState;
    }

    void Roller::control(pros::Controller master)
    {
        static bool automatic{false};
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
            automatic = !automatic;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {

            if (automatic)
                setState(RollerState::TurningToColor);
            else
                setState(RollerState::Turning);
        }
        else
            setState(RollerState::Idle);
    }

    void Roller::turnToColor()
    {
        runRoller(-100);
        waitFor([=]()
                { return !isCorrectColor(); },
                2_s);
        runRoller();
        waitFor([=]()
                { return isCorrectColor(); },
                2_s);
        waitFor([=]()
                { return !isCorrectColor(); },
                2_s);
        stopRoller();
        setState(RollerState::Idle);
    }

    void Roller::runRoller(int speed)
    {
        motor->move_velocity(speed);
    }

    void Roller::stopRoller()
    {
        motor->move(0);
    }

    bool Roller::isCorrectColor()
    {
        if (opticalA->get_proximity() > opticalB->get_proximity())
            return autonSelector->getColor() == Color::Red ? opticalA->get_hue() < blueHue : opticalA->get_hue() > redHue;
        return autonSelector->getColor() == Color::Red ? opticalB->get_hue() < blueHue : opticalB->get_hue() > redHue;
    }

    SPRoller SPRollerBuilder::build() const
    {
        return std::make_shared<Roller>(std::make_unique<pros::Motor>(motorPort, gearset),
                                        std::make_unique<pros::Optical>(opticalAPort),
                                        std::make_unique<pros::Optical>(opticalBPort),
                                        autonSelector,
                                        redHue,
                                        blueHue);
    }

    SPRollerBuilder SPRollerBuilder::withMotor(int8_t iPort,
                                               const pros::motor_gearset_e_t &iGearset)
    {
        motorPort = iPort;
        gearset = iGearset;
        return *this;
    }

    SPRollerBuilder SPRollerBuilder::withOpticals(int8_t portA, int8_t portB)
    {
        opticalAPort = portA;
        opticalBPort = portB;
        return *this;
    }

    SPRollerBuilder SPRollerBuilder::withAutonSelector(SPAutonSelector iAutonSelector)
    {
        autonSelector = iAutonSelector;
        return *this;
    }

    SPRollerBuilder SPRollerBuilder::withRedHue(int iRedHue)
    {
        redHue = iRedHue;
        return *this;
    }

    SPRollerBuilder SPRollerBuilder::withBlueHue(int iBlueHue)
    {
        blueHue = iBlueHue;
        return *this;
    }
}