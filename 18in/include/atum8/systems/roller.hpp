#pragma once

#include "atum8/misc/constants.hpp"
#include "atum8/gui/autonSelector.hpp"
#include "atum8/misc/task.hpp"
#include "pros/misc.hpp"

namespace atum8
{
    class Roller : public Task
    {
    public:
        enum class RollerState
        {
            TurningToColor,
            Turning,
            Idle
        };

        Roller(UPMotor iMotor,
               UPOptical iOpticalA,
               UPOptical iOpticalB,
               SPAutonSelector iAutonSelector,
               int iRedHue,
               int iBlueHue);


        void setState(const RollerState &iRollerState);

        RollerState getState() const;

        void control(pros::Controller master);



    private:
        TaskFn rollerControlTask();
        void turnToColor();
        void runRoller(int velocity = 100);
        void stopRoller();
        bool isCorrectColor();
        UPMotor motor;
        UPOptical opticalA;
        UPOptical opticalB;
        SPAutonSelector autonSelector;
        int redHue;
        int blueHue;
        RollerState rollerState;
    };

    using UPRoller = std::unique_ptr<Roller>;
    using SPRoller = std::shared_ptr<Roller>;

    /**
     * @brief Provides a builder for the roller.
     *
     */
    class SPRollerBuilder
    {
    public:
        /**
         * @brief Constructs the roller object.
         *
         * @return SPRoller
         */
        SPRoller build() const;

        /**
         * @brief Roller configured with this motor.
         *
         * @param iPort
         * @param iGearset
         * @return SPRollerBuilder
         */
        SPRollerBuilder withMotor(int8_t iPort,
                                  const pros::motor_gearset_e_t &iGearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        SPRollerBuilder withOpticals(int8_t portA, int8_t portB);

        SPRollerBuilder withAutonSelector(SPAutonSelector iAutonSelector);

        SPRollerBuilder withRedHue(int iRedHue);

        SPRollerBuilder withBlueHue(int iBlueHue);

    private:
        int8_t motorPort;
        pros::motor_gearset_e_t gearset;
        int8_t opticalAPort;
        int8_t opticalBPort;
        SPAutonSelector autonSelector;
        int redHue;
        int blueHue;
    };
}