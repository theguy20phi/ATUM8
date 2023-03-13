#include "atum8/systems/drive.hpp"
#include "atum8/globals.hpp"
#include "main.h"

namespace atum8 {
Pid linearController(0, 0, 0, .5, .05);
Pid turnController(0, 0, 0, 1, .05);

void Drive::controller(){};

void Drive::setRightPower(double power) {
  rightFrontDrive.move_voltage(power);
  rightMiddleDrive.move_voltage(power);
  rightBackDrive.move_voltage(power);
}

void Drive::setLeftPower(double power) {
  leftFrontDrive.move_voltage(power);
  leftMiddleDrive.move_voltage(power);
  leftBackDrive.move_voltage(power);
}

double Drive::getRightEncoderValues() {
  return (
      (driveGearRatio) * // drive gear ratio
      ((rightFrontDrive.get_position() + rightMiddleDrive.get_position() + rightBackDrive.get_position()) ) / 3);
}

double Drive::getLeftEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((leftFrontDrive.get_position() + leftMiddleDrive.get_position() +leftBackDrive.get_position())) / 3);
}

double Drive::getEncoderAverages() {
  return (Drive::getRightEncoderValues() +
          Drive::getLeftEncoderValues()) /
         2;
}
double rpmToPower(double rpm) { return (rpm * 12000) / 200; };

void Drive::move(double inches, double rpm, double acceleration, bool dift, double secThreshold) {
    linearController.setMaxOutput(rpmToPower(rpm));
    while(true) {
        power = linearController.getOutput((2 * M_PI * encoderWheelRadius * getEncoderAverages()) / 360, inches);

        if(linearController.isSettled())
          break;
        else
          continue;

        setRightPower(power);
        setLeftPower(power);
    }

}

void Drive::turn(double angle, double rpm, double acceleration, double secThreshold) {
    turnController.setMaxOutput(rpmToPower(rpm));
  while (true) {
    power = turnController.getOutput(getImuSensorAverages(), angle);

    if(turnController.isSettled())
      break;
    else
      continue;

    setRightPower(-power);
    setLeftPower(power);
  }
}
} // namespace atum8