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
                prevTime = pros::millis() * okapi::millisecond;
            }
            piston->set_value(1);
            pros::delay(stdDelay);
        }
    }

    void Intake::runIntake(int speed)
    {
        motor->move(speed);
    }

    void Intake::stopIntake()
    {
        motor->move(0);
    }

    void Intake::shoot(int iShooting, const okapi::QTime &iTimeout)
    {
        shooting = iShooting;
        timeout = iTimeout;
        prevTime = pros::millis() * okapi::millisecond;
    }

    bool Intake::isShooting() const
    {
        return shooting;
    }

    bool Intake::shouldShoot()
    {
        if(shooting <= 0) return false;
        okapi::QTime currentTime{pros::millis() * okapi::millisecond};
        if(timeout == 0_s) return true;
        return flywheel->readyToFire() || (currentTime - prevTime >= timeout);
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