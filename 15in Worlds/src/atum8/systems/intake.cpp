#include "atum8/systems/intake.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"

namespace atum8 {
void Intake::taskFn() {
    intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  while (true) {
    controller();
    pros::delay(10);
  }
}

void Intake::controller() {
    if(Chris.get_digital_new_press(DIGITAL_LEFT))
        isDiskMode = !isDiskMode;

    if(isDiskMode == true)
        diskControlls();
    else 
        rollerControlls();
}

void Intake::diskControlls() {
  if (Chris.get_digital(DIGITAL_L1)) {
    intakeMotors.move_voltage(12000);
    buttonDuration = 0;
  } else if (Chris.get_digital(DIGITAL_L2)) {
    buttonDuration += 10;
    if (buttonDuration < longPress) {
      intakeMotors.move_voltage(0);
    } else {
      intakeMotors.move_voltage(-12000);
    }
  } else {
    buttonDuration = 0;
  }
}

void Intake::rollerControlls() {
    if(Chris.get_digital(DIGITAL_L1))
        intakeMotors.move_voltage(12000);
    else if(Chris.get_digital(DIGITAL_L2))
        intakeMotors.move_voltage(-12000);
    else
        intakeMotors.move_voltage(0);
}

} // namespace atum8
