#include "atum8/catapult.hpp"
#include "main.h"

namespace atum8 {

void Catapult::down() {
  rightCatapultMotor.move_voltage(12000);
  leftCatapultMotor.move_voltage(12000);
};

void Catapult::up() {
  rightCatapultMotor.move_voltage(-12000);
  leftCatapultMotor.move_voltage(-12000);
}

void Catapult::stop() {
  rightCatapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  leftCatapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightCatapultMotor.move_voltage(0);
  leftCatapultMotor.move_voltage(0);
}

void Catapult::downUntilPrimed() {
  while (!catapultStop.get_value()) {
    down();
  }
  stop();
}

void Catapult::shoot() {
  double intakeEfficiency =
      (rightIntakeMotor.get_efficiency() + leftIntakeMotor.get_efficiency()) /
      2;
  double intakeEfficiencyThreshold;

  //if (intakeEfficiency < intakeEfficiencyThreshold) {
  //  rightIntakeMotor.move_voltage(0);
  //  leftIntakeMotor.move_voltage(0);
  //} else {

    rightCatapultMotor.move_voltage(12000);
    leftCatapultMotor.move_voltage(12000);
    pros::delay(200);
  //}
}

void Catapult::controller() {
  static bool manualOverride{false};

  if (Chris.get_digital_new_press(DIGITAL_B))
    manualOverride = !manualOverride;

  if (!manualOverride) {
    if (!catapultStop.get_value()) {
      down();
    } else {
      if (Chris.get_digital(DIGITAL_R1)) {
        down();
      } else {
        stop();
      }
    }
  } else {
    if (Chris.get_digital(DIGITAL_R1)) {
      down();
    } else if (Chris.get_digital(DIGITAL_R2)) {
      up();
    } else {
      stop();
    }
  }
}
} // namespace atum8