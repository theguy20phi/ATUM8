#include "intake.hpp"

namespace atum8
{
    Intake::Intake(UPMotor iMotor,
                   UPADIDigitalOut iPiston,
                   const SPFlywheel &iFlywheel,
                   int iShotDelay) : motor{std::move(iMotor)},
                                     piston{std::move(iPiston)},
                                     flywheel{iFlywheel},
                                     shotDelay{iShotDelay}
    {
    }

    void Intake::taskFn()
    {
        while (true)
        {
            if (shouldShoot())
            {
                piston->set_value(0);
                pros::delay(shotDelay);
                piston->set_value(1);
                pros::delay(shotDelay);
                shooting--;
            }
            piston->set_value(1);
            pros::delay(stdDelay);
        }
    }

    void Intake::runIntake(int speed)
    {
        motor->move(speed);
    }

    void Intake::shoot(int iShooting, bool iFlywheelBlocks)
    {

        shooting = iShooting;
        flywheelBlocks = iFlywheelBlocks;
    }

    bool Intake::isShooting() const
    {
        return shooting;
    }

    bool Intake::shouldShoot() {
        if(flywheelBlocks)
            return flywheel->readyToFire() && shooting;
        return shooting;
    }

    SPIntake SPIntakeBuilder::build() const
    {
        return std::make_shared<Intake>(std::make_unique<pros::Motor>(motorPort, gearset),
                                        std::make_unique<pros::ADIDigitalOut>(pistonPort),
                                        flywheel,
                                        shotDelay);
    }

    SPIntakeBuilder SPIntakeBuilder::withMotor(int iPort,
                                               const pros::motor_gearset_e_t &iGearset)
    {
        motorPort = iPort;
        gearset = iGearset;
        return *this;
    }

    SPIntakeBuilder SPIntakeBuilder::withPiston(int iPort)
    {
        pistonPort = iPort;
        return *this;
    }

    SPIntakeBuilder SPIntakeBuilder::withFlywheel(const SPFlywheel &iFlywheel)
    {
        flywheel = iFlywheel;
        return *this;
    }

    SPIntakeBuilder SPIntakeBuilder::withShotDelay(int iShotDelay)
    {
        shotDelay = iShotDelay;
        return *this;
    }
}