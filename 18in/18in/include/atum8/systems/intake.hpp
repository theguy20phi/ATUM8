#pragma once

#include "atum8/constants.hpp"
#include "atum8/task.hpp"

namespace atum8
{
    class Intake : public Task
    {
    public:
        Intake(UPMotor iMotor,
               UPADIDigitalOut iPiston,
               UPADIAnalogIn iLowLineTracker,
               UPADIAnalogIn iMiddleLineTracker,
               UPADIAnalogIn iHighLineTracker,
               int iLineTrackerThreshold);

        void taskFn();

        void runIntake(int speed = 127);

        void shoot();

        int getNumberOfDisks() const;

        void setLineTrackerThreshold(int iLineTrackerThreshold);

        int getLineTrackerThreshold() const;

    private:
        UPMotor motor;
        UPADIDigitalOut piston;
        UPADIAnalogIn lowLineTracker;
        UPADIAnalogIn middleLineTracker;
        UPADIAnalogIn highLineTracker;
        int lineTrackerThreshold;
        bool shooting{false};
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

        SPIntakeBuilder withLineTrackers(int iLowPort, int iMiddlePort, int iHighPort);

        SPIntakeBuilder withLineTrackerThreshold(int iLineTrackerThreshold);

    private:
        int motorPort;
        pros::motor_gearset_e_t gearset;
        int pistonPort;
        int lowPort;
        int middlePort;
        int highPort;
        int lineTrackerThreshold;
    };
}