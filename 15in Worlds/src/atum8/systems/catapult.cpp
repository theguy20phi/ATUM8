#include "atum8/systems/catapult.hpp"
#include "main.h"

namespace atum8 {
    void Catapult::taskFn() {
        catapultMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
        while(true) {
            controller();
            pros::delay(10);
        }
    }

    void Catapult::controller(){
        if(Chris.get_digital(DIGITAL_R1))
            catapultMotors.move_voltage(12000);
        else if(Chris.get_digital(DIGITAL_R2))
            catapultMotors.move_voltage(-12000);
        else
            catapultMotors.move_voltage(0);
    }
}