#include "atum8/driveHelpers.hpp"
#include "main.h"

namespace atum8 {

double DriveHelpers::rpmToPower(double rpm) { return (rpm * 12000) / 200; };

void DriveHelpers::setRightPower(double power) {
  // std::cout<< "setting right power:" << power << std::endl;
  rightFrontDrive.move_voltage(power);
  rightMiddleDrive.move_voltage(power);
  rightBackDrive.move_voltage(power);
}

void DriveHelpers::setLeftPower(double power) {
  // std::cout<< "setting left power:" << power <<std::endl;
  leftFrontDrive.move_voltage(power);
  leftMiddleDrive.move_voltage(power);
  leftBackDrive.move_voltage(power);
}

void DriveHelpers::setMoveRight(double power){
  rightFrontDrive.move(power);
  rightMiddleDrive.move(power);
  rightBackDrive.move(power);
}

void DriveHelpers::setMoveLeft(double power){
  leftFrontDrive.move(power);
  leftMiddleDrive.move(power);
  leftBackDrive.move(power);
}

void DriveHelpers::rightAcceleration(double rightTarget,
                                     double accelerationStepUp) {
  double step;

  if (fabs(rightPower) < fabs(rightTarget))
    step = accelerationStepUp;
  else
    step = accelerationStepDown;

  if (rightTarget > rightPower + step)
    rightPower += step;
  else if (rightTarget < rightPower - step)
    rightPower -= step;
  else
    rightPower = rightTarget;

  setRightPower(rightPower);
}

void DriveHelpers::leftAcceleration(double leftTarget,
                                    double accelerationStepUp) {
  double step;

  if (fabs(leftPower) < fabs(leftTarget))
    step = accelerationStepUp;
  else
    step = accelerationStepDown;

  if (leftTarget > leftPower + step)
    leftPower += step;
  else if (leftTarget < leftPower - step)
    leftPower -= step;
  else
    leftPower = leftTarget;
  setLeftPower(leftPower);
};

void DriveHelpers::setDriveBrakeMode(const std::string brakeMode) {
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
  // std::cout << "brake mode set to: " << brakeMode << std::endl;
}

void DriveHelpers::resetEncoders() {
  rightFrontDrive.tare_position();
  rightMiddleDrive.tare_position();
  rightBackDrive.tare_position();

  leftFrontDrive.tare_position();
  leftMiddleDrive.tare_position();
  leftBackDrive.tare_position();
}

double DriveHelpers::getRightEncoderValues() {
  return (
      (driveGearRatio) * // drive gear ratio
      ((rightFrontDrive.get_position() + rightMiddleDrive.get_position() + rightBackDrive.get_position()) ) / 3);
}

double DriveHelpers::getLeftEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((leftFrontDrive.get_position() + leftMiddleDrive.get_position() +leftBackDrive.get_position())) / 3);
}

double DriveHelpers::getEncoderAverages() {
  return (DriveHelpers::getRightEncoderValues() +
          DriveHelpers::getLeftEncoderValues()) /
         2;
}

void DriveHelpers::resetImuSensors(){
  imuSensorAlpha.tare();
  imuSensorBeta.tare();
  imuSensorCharlie.tare();
}

double DriveHelpers::getImuSensorAverages() {
  return ((imuSensorAlpha.get_rotation() + imuSensorBeta.get_rotation() + imuSensorCharlie.get_rotation()) / 3);
}
} // namespace atum8