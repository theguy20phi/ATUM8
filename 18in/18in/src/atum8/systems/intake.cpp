#include "intake.hpp"

namespace atum8
{
    Intake::Intake(UPMotor iMotor,
                   UPADIDigitalOut iPiston,
                   UPADIAnalogIn iLowLineTracker,
                   UPADIAnalogIn iMiddleLineTracker,
                   UPADIAnalogIn iHighLineTracker,
                   int iLineTrackerThreshold) : motor{std::move(iMotor)},
                                                piston{std::move(iPiston)},
                                                lowLineTracker{std::move(iLowLineTracker)},
                                                middleLineTracker{std::move(iMiddleLineTracker)},
                                                highLineTracker{std::move(iHighLineTracker)},
                                                lineTrackerThreshold{iLineTrackerThreshold}
    {
    }

    void Intake::taskFn()
    {
        while (true)
        {
            if (shooting)
            {
                piston->set_value(0);
                waitFor([this]()
                        { return lowLineTracker->get_value() >= lineTrackerThreshold; },
                        1_s);
                shooting = false;
            }
            piston->set_value(1);
            pros::delay(stdDelay);
        }
    }

    void Intake::runIntake(int speed)
    {
        motor->move(speed);
    }

    void Intake::shoot()
    {
        shooting = true;
    }

    bool Intake::isShooting() const {
        return shooting;
    }

    int Intake::getNumberOfDisks() const
    {
        if (lowLineTracker->get_value() < lineTrackerThreshold)
        {
            if (middleLineTracker->get_value() < lineTrackerThreshold)
            {
                if (highLineTracker->get_value() < lineTrackerThreshold)
                    return 3;
                return 2;
            }
            return 1;
        }
        return 0;
    }

    void Intake::setLineTrackerThreshold(int iLineTrackerThreshold)
    {
        lineTrackerThreshold = iLineTrackerThreshold;
    }

    int Intake::getLineTrackerThreshold() const
    {
        return lineTrackerThreshold;
    }

    SPIntake SPIntakeBuilder::build() const
    {
        return std::make_shared<Intake>(std::make_unique<pros::Motor>(motorPort, gearset),
                                        std::make_unique<pros::ADIDigitalOut>(pistonPort),
                                        std::make_unique<pros::ADIAnalogIn>(lowPort),
                                        std::make_unique<pros::ADIAnalogIn>(middlePort),
                                        std::make_unique<pros::ADIAnalogIn>(highPort),
                                        lineTrackerThreshold);
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

    SPIntakeBuilder SPIntakeBuilder::withLineTrackers(int iLowPort, int iMiddlePort, int iHighPort)
    {
        lowPort = iLowPort;
        middlePort = iMiddlePort;
        highPort = iHighPort;
        return *this;
    }

    SPIntakeBuilder SPIntakeBuilder::withLineTrackerThreshold(int iLineTrackerThreshold)
    {
        lineTrackerThreshold = iLineTrackerThreshold;
        return *this;
    }
}