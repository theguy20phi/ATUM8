#include "atum8/systems/drive.hpp"
#include <algorithm>
#include <cmath>
#include <math.h>
#include "main.h"

namespace atum8 {
Pid linearController(800, 1, 8.8, 0.5, .05);
Pid turnController(270, 0.8, 0, 1, .05);
Pid linearMTPController(800, 1, 0, 2, .05);
Pid turnMTPController(300, 0, 0, 1, .05);
Pid turnTPController(300, 0, 0, 1, .05);


void Drive::taskFn() {
  setDriveBrakeMode("BRAKE");
  while(true) {
    tankDrive();
    pros::delay(10);
  }
}

void Drive::tankDrive() {
  rightDriveMotors.move(Chris.get_analog(ANALOG_RIGHT_Y));
  leftDriveMotors.move(Chris.get_analog(ANALOG_LEFT_Y));
};

void Drive::arcadeDrive() {
  rightDriveMotors.move(Chris.get_analog(ANALOG_LEFT_Y) - Chris.get_analog(ANALOG_RIGHT_X));
  leftDriveMotors.move(Chris.get_analog(ANALOG_LEFT_Y) + Chris.get_analog(ANALOG_RIGHT_X));
}

void Drive::movePID(const double inches, const double rpm, const double acceleration, const bool dift, const double secThreshold) {
  reset();
  linearController.setMaxOutput(utility::rpmToPower(rpm));
  while (true) {
    output = linearController.getOutput(
        (2 * M_PI * encoderWheelRadius * getEncoderAverages()) / 360, inches);

    setRightPower(SlewRate::getOutput(getRightPower(), output, acceleration));
    setLeftPower(SlewRate::getOutput(getLeftPower(), output, acceleration));
    

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
    output = turnController.getOutput(getImuSensorAverages(), angle);

    setRightPower(SlewRate::getOutput(getRightPower(), -output, acceleration));
    setLeftPower(SlewRate::getOutput(getLeftPower(), output, acceleration));

    if (turnController.isSettled())
      break;
    if (msCounter / 1000 > secThreshold)
      break;

    msCounter+=10;
    pros::delay(10);
  }
  reset();
}

void Drive::moveToPoint(const double desiredX, const double desiredY, const double linearRpm, const double turnRpm, const double secThreshold) {
  reset();
  linearMaxPower = utility::rpmToPower(linearRpm);
  turnMaxPower = utility::rpmToPower(turnRpm);

  while(true) {
    errorX = desiredX - globalX;
    errorY = desiredY - globalY;

    linearError = hypot(errorX, errorY);
    turnError = utility::constrain180(utility::convertRadianToDegree(atan2(errorX, errorY))-globalHeadingInDegrees);
    linearOutput = linearMTPController.getOutput(linearError);

    headingScaleFactor = cos(utility::convertDegreeToRadian(turnError));
    linearOutput*=headingScaleFactor;
    turnError = utility::constrain90(turnError);
    turnOutput = turnMTPController.getOutput(turnError);

    if (linearError < 2)
      turnOutput = 0;

    linearOutput = utility::clamp(linearOutput, -fabs(headingScaleFactor) * linearMaxPower, fabs(headingScaleFactor) * linearMaxPower);
    turnOutput = utility::clamp(turnOutput, -turnMaxPower, turnMaxPower);

    if(linearMTPController.isSettled())
      break;
    if (msCounter / 1000 > secThreshold)
      break;
    
    setRightPower(linearOutput - turnOutput);
    setLeftPower(linearOutput + turnOutput);

    pros::delay(10);
  }
  reset();
}

void Drive::turnToPoint(const double desiredX, const double desiredY, const double rpm, const double secThreshold) {
  reset();
  turnMaxPower = utility::rpmToPower(rpm);
  while(true) {
    errorX = desiredX - globalX;
    errorY = desiredY - globalY;
    output = turnTPController.getOutput(utility::constrain180(utility::convertRadianToDegree(atan2(errorX, errorY)) - globalHeadingInDegrees));
    output = utility::clamp(turnOutput, -turnMaxPower, turnMaxPower);

    
    //if(turnTPController.isSettled())
      //break;
    //if (msCounter / 1000 > secThreshold)
      //break;

    setRightPower(-output);
    setLeftPower(output);

    pros::delay(10);
  }
  reset();
}

void Drive::turnToAngle(const double angle, const double rpm, const double secThreshold){
  reset();
  turnMaxPower = utility::rpmToPower(rpm);
  while(true) {
    output = turnController.getOutput(utility::constrain180(angle - globalHeadingInDegrees));
    setRightPower(-output);
    setLeftPower(output);
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
  output = 0;
  msCounter = 0;
  linearMaxPower = 0;
  turnMaxPower = 0;
  errorX = 0;
  errorY = 0;
  linearError = 0;
  turnError = 0;
  linearOutput = 0;
  turnOutput = 0;
  headingScaleFactor = 0;

  setRightPower(0);
  setLeftPower(0);
  linearController.reset();
  turnController.reset();
  linearMTPController.reset();
  turnMTPController.reset();
  resetImuSensors();
  resetEncoders();
  setDriveBrakeMode("COAST");
}
} // namespace atum8