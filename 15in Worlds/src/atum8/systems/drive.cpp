#include "atum8/systems/drive.hpp"
#include "atum8/globals.hpp"
#include "atum8/slewRate.hpp"
#include "main.h"

namespace atum8 {
Pid linearController(800, 1, 8.8, .5, .05);
Pid turnController(270, 0.8, 0, 1, .05);

void Drive::controller() {
  setDriveBrakeMode("BRAKE");
  rightFrontDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightMiddleDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightBackDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));

  leftFrontDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftMiddleDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftBackDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
};

double rpmToPower(double rpm) { return (rpm * 12000) / 200; };

void Drive::move(double inches, double rpm, double acceleration, bool dift,
                 double secThreshold) {
  reset();
  linearController.setMaxOutput(rpmToPower(rpm));
  while (true) {
    power = linearController.getOutput(
        (2 * M_PI * encoderWheelRadius * getEncoderAverages()) / 360, inches);
        
    setRightPower(SlewRate::getOutput(getRightPower(), power, acceleration));
    setLeftPower(SlewRate::getOutput(getLeftPower(), power, acceleration));

    if (linearController.isSettled())
      break;
    if (msCounter / 1000 > secThreshold)
      break;

    msCounter+=10;
    pros::delay(10);
  }
  reset();
}

void Drive::turn(double angle, double rpm, double acceleration,
                 double secThreshold) {
  reset();
  turnController.setMaxOutput(rpmToPower(rpm));
  while (true) {
    power = turnController.getOutput(getImuSensorAverages(), angle);

    setRightPower(SlewRate::getOutput(getRightPower(), -power, acceleration));
    setLeftPower(SlewRate::getOutput(getLeftPower(), power, acceleration));

    if (turnController.isSettled())
      break;
    if (msCounter / 1000 > secThreshold)
      break;

    msCounter+=10;
    pros::delay(10);
  }
  reset();
}

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

double Drive::getRightPower() {
  return ((rightFrontDrive.get_voltage() + rightMiddleDrive.get_voltage() +
           rightBackDrive.get_voltage()) /
          3);
}

double Drive::getLeftPower() {
  return ((leftFrontDrive.get_voltage() + leftMiddleDrive.get_voltage() +
           leftBackDrive.get_voltage()) /
          3);
}

void Drive::setDriveBrakeMode(const std::string brakeMode) {
  if (brakeMode == "COAST") {
    rightFrontDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    rightMiddleDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_COAST);

    leftFrontDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    leftMiddleDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_COAST);
  }

  if (brakeMode == "BRAKE") {
    rightFrontDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMiddleDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);

    leftFrontDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftMiddleDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
  }

  if (brakeMode == "HOLD") {
    rightFrontDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightMiddleDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_HOLD);

    leftFrontDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    leftMiddleDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
  }
}

double Drive::getRightEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((rightFrontDrive.get_position() + rightMiddleDrive.get_position() +
            rightBackDrive.get_position())) /
          3);
}

double Drive::getLeftEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((leftFrontDrive.get_position() + leftMiddleDrive.get_position() +
            leftBackDrive.get_position())) /
          3);
}

double Drive::getEncoderAverages() {
  return (Drive::getRightEncoderValues() + Drive::getLeftEncoderValues()) / 2;
}

void Drive::resetEncoders() {
  rightFrontDrive.tare_position();
  rightMiddleDrive.tare_position();
  rightBackDrive.tare_position();

  leftFrontDrive.tare_position();
  leftMiddleDrive.tare_position();
  leftBackDrive.tare_position();
}

void Drive::reset() {
  power = 0;
  msCounter = 0;

  setRightPower(0);
  setLeftPower(0);
  linearController.reset();
  turnController.reset();
  resetImuSensors();
  resetEncoders();
  setDriveBrakeMode("COAST");
}
} // namespace atum8