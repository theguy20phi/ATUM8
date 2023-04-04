#include "atum8/systems/drive.hpp"
#include <cmath>
#include <math.h>
#include "main.h"

namespace atum8 {
Pid linearController(800, 1, 8.8, 0.5, .05);
Pid turnController(270, 0.8, 0, 1, .05);
Pid coordinateController(800, 0, 0, 0.5, 0.05);
Pid headingController(300, 0, 0, 1, 0.05);


void Drive::taskFn() {
  while(true) {
    controller();
    pros::delay(10);
  }
}

void Drive::controller() {
  setDriveBrakeMode("COAST");
  rightFrontTopDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightFrontBotDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightBackDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));

  leftFrontTopDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftFrontBotDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftBackDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
};



void Drive::movePID(const double inches, const double rpm, const double acceleration, const bool dift, const double secThreshold) {
  reset();
  linearController.setMaxOutput(utility::rpmToPower(rpm));
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

void Drive::turnPID(const double angle, const double rpm, const double acceleration, const double secThreshold) {
  reset();
  turnController.setMaxOutput(utility::rpmToPower(rpm));
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

void Drive::moveToPoint(const double desiredX, const double desiredY, const double desiredHeading, const double coordinateRpm, const double headingRpm, const double secThreshold) {
  coordinateController.reset();
  headingController.reset();
  reset();
  double coordinateMaxPower = utility::rpmToPower(coordinateRpm);
  double headingMaxPower = utility::rpmToPower(headingRpm);
  coordinateController.setMaxOutput(coordinateMaxPower);
  headingController.setMaxOutput(headingMaxPower);

  while(true) {
    double errorX = desiredX - globalX;
    double errorY = desiredY - globalY;
    double coordinateError = hypot(errorX, errorY);
    double headingError = utility::reduce_negative_180_to_180(utility::convertRadianToDegree(atan2(errorX, errorY))-globalHeadingInDegrees);
    double coordinatePower = coordinateController.getOutput(coordinateError);

    double headingScaleFactor = cos(utility::convertDegreeToRadian(headingError));
    coordinatePower*=headingScaleFactor;
    headingError = utility::reduce_negative_90_to_90(headingError);
    double headingPower = headingController.getOutput(headingError);

    if (coordinateError < 2)
      headingPower = 0;

    coordinatePower = utility::clamp(coordinatePower, -fabs(headingScaleFactor) * coordinateMaxPower, fabs(headingScaleFactor) * coordinateMaxPower);
    headingPower = utility::clamp(headingPower, -headingMaxPower, headingMaxPower);

    setRightPower(coordinatePower - headingPower);
    setLeftPower(coordinatePower + headingPower);

    pros::delay(10);
  }
  reset();
}

void Drive::moveToReference(const double desiredX, const double desiredY, const double desiredHeading) {
  coordinateController.reset();
  headingController.reset();
  coordinateController.setMaxOutput(12000);
  headingController.setMaxOutput(6000);
  reset();
  while(true) {
    double errorX = desiredX - globalX;
    double errorY = desiredY - globalY;
    double distanceError = hypot(errorX, errorY);
    double headingError = utility::reduce_negative_180_to_180(90 - atan2(errorY, errorX) - globalHeadingInDegrees);

    double distancePower = coordinateController.getOutput(distanceError);
    double headingPower = headingController.getOutput(headingError);

    if(distanceError < 2)
      headingPower = 0;

    setRightPower(distancePower - headingPower);
    setLeftPower(distancePower + headingPower);

    pros::delay(10);
  }
  reset();
}

void Drive::setRightPower(double power) {
  rightFrontTopDrive.move_voltage(power);
  rightFrontBotDrive.move_voltage(power);
  rightBackDrive.move_voltage(power);
}

void Drive::setLeftPower(double power) {
  leftFrontTopDrive.move_voltage(power);
  leftFrontBotDrive.move_voltage(power);
  leftBackDrive.move_voltage(power);
}

double Drive::getRightPower() {
  return ((rightFrontTopDrive.get_voltage() + rightFrontBotDrive.get_voltage() +
           rightBackDrive.get_voltage()) /
          3.0);
}

double Drive::getLeftPower() {
  return ((leftFrontTopDrive.get_voltage() + leftFrontBotDrive.get_voltage() +
           leftBackDrive.get_voltage()) /
          3.0);
}

void Drive::setDriveBrakeMode(const std::string brakeMode) {
  if (brakeMode == "COAST") {
    rightFrontTopDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    rightFrontBotDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_COAST);

    leftFrontTopDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    leftFrontBotDrive.set_brake_mode(MOTOR_BRAKE_COAST);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_COAST);
  }

  if (brakeMode == "BRAKE") {
    rightFrontTopDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightFrontBotDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);

    leftFrontTopDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftFrontBotDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_BRAKE);
  }

  if (brakeMode == "HOLD") {
    rightFrontTopDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightFrontBotDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightBackDrive.set_brake_mode(MOTOR_BRAKE_HOLD);

    leftFrontTopDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    leftFrontBotDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
    leftBackDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
  }
}

double Drive::getRightEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((rightFrontTopDrive.get_position() + rightFrontBotDrive.get_position() +
            rightBackDrive.get_position())) /
          3);
}

double Drive::getLeftEncoderValues() {
  return ((driveGearRatio) * // drive gear ratio
          ((leftFrontTopDrive.get_position() + leftFrontBotDrive.get_position() +
            leftBackDrive.get_position())) /
          3);
}

double Drive::getEncoderAverages() {
  return (Drive::getRightEncoderValues() + Drive::getLeftEncoderValues()) / 2;
}

void Drive::resetEncoders() {
  rightFrontTopDrive.tare_position();
  rightFrontBotDrive.tare_position();
  rightBackDrive.tare_position();

  leftFrontTopDrive.tare_position();
  leftFrontBotDrive.tare_position();
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