#include "atum8/systems/intake.hpp"
#include "main.h"
#include "pros/motors.h"

namespace atum8 {
    void Intake::taskFn() {
        while(true) {
            controller();
            pros::delay(10);
        }
    }

    void Intake::controller(){
        static int buttonDuration;
        const short int longPress{ 250 };

        if (Chris.get_digital(DIGITAL_L1)) {
            intakeMotors.move_voltage(12000);
            buttonDuration = 0;
        }
        else if(Chris.get_digital(DIGITAL_L2)) {
            buttonDuration += 10;
            if(buttonDuration < longPress) {
                intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
                intakeMotors.move_voltage(0);
            } else {
                intakeMotors.move_voltage(-12000);
            }
        } 
        else {
            buttonDuration = 0;
        }
        }
        
} // namespace atum8
