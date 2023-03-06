#include "atum8/intake.hpp"
#include "main.h"
#include <chrono>
#include <ratio>

namespace atum8 {

void Intake::setIntakePower(double power) {
  rightIntakeMotor.move_voltage(power);
  leftIntakeMotor.move_voltage(power);
}

void Intake::in() { setIntakePower(12000); }

void Intake::out() { setIntakePower(-12000); }

void Intake::stop() {
  rightIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  leftIntakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  setIntakePower(0);
}



void Intake::setRollerToRed() {

  while (opticalSensor.get_hue() <
         blueRollerHue -
             rollerColorThreshold)

  {
      std::cout<< opticalSensor.get_hue() << std::endl;
    setRightPower(-1200);
    setLeftPower(-1200);

    setIntakePower(-12000);
  }
  setIntakePower(3000);
  pros::delay(500);
  stop();
  setRightPower(0);
  setLeftPower(0);
}

void Intake::setRollerToBlue() {
  while (opticalSensor.get_hue() >
         redRollerHue +
             rollerColorThreshold)
  {

    setRightPower(-1000);
    setLeftPower(-1000);

    setIntakePower(-12000);
  }
  setIntakePower(3000);
  pros::delay(500);
  stop();
  setRightPower(0);
  setLeftPower(0);
}

void Intake::controller() {
  static int buttonDuration;
  int longPress{250};

  if (Chris.get_digital(DIGITAL_L1)) {
    in();
    buttonDuration = 0;
  }
  else if (Chris.get_digital(DIGITAL_L2)) {

    buttonDuration += 10;
    if (buttonDuration < longPress) {
      stop();
    } else {
      out();
    }
  }else {
    buttonDuration = 0;
  }
}
} // namespace atum8