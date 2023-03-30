#include "atum8/systems/drive.hpp"
#include "atum8/globals.hpp"
#include <math.h>
#include "main.h"

namespace atum8 {
Pid linearController(800, 1, 8.8, .5, .05);
Pid turnController(270, 0.8, 0, 1, .05);
Pid linearMTPController(800, 0, 0, 0.5, 0.05);
Pid turnMTPController(270, 0.8, 0, 1, .05);

void Drive::taskFn() {
  while(true) {
    controller();
    pros::delay(10);
  }
}

void Drive::controller() {
  setDriveBrakeMode("COAST");
  rightFrontDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightMiddleDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));
  rightBackDrive.move(Chris.get_analog(ANALOG_RIGHT_Y));

  leftFrontDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftMiddleDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
  leftBackDrive.move(Chris.get_analog(ANALOG_LEFT_Y));
};

double rpmToPower(double rpm) { return (rpm * 12000) / 200; };

int sgn(double num){
  return (num < 0) ? -1 : ((num > 0) ? 1 : 0); // Returns -1 if num is negative, and 1 if num is HIV positive.
}

int16_t radian_to_degrees_converter(const double angle) { return angle * 180 / M_PI; } // convert radian to degrees
int16_t degrees_to_radians_converter(const double angle){ return angle * M_PI / 180; } // Convert degrees to radian

double Drive::findMinAngle(const double targetHeading, const double currentHeading){
  double turnAngle = targetHeading - globalHeadingInDegrees;
  if (turnAngle > 180 || turnAngle < -180){ turnAngle = turnAngle - (sgn(turnAngle) * 360); }
  return turnAngle;
}




void Drive::moveToReference(const double targetX, const double targetY, const double targetHeading, const double radiusOfArc, const double rpm, const double acceleration, const double secThreshold) {
  double linearMTPkP {10};
  double turnMTPkP { 10 };
  double closeTollerance { 1 };
  double targetFinalTollerance { 0.1 };
  bool closeToTarget{ false };

  while(true) {
    absTargetAngle = atan2f(targetX - globalX, targetY - globalY) * 180 / M_PI;
    if(absTargetAngle < 0)
      absTargetAngle += 360;
    distance = sqrt(pow(targetX - globalX, 2) + pow(targetY - globalY, 2));
    alpha = findMinAngle(absTargetAngle, targetHeading);
    errorTerm = findMinAngle(absTargetAngle, globalHeadingInDegrees);
    beta = atan(radiusOfArc / distance) * 180 / M_PI;

    if(alpha < 0)
      beta = -beta;
    if(fabs(alpha) < fabs(beta))
      turnError = errorTerm + alpha;
    else
      turnError = errorTerm + beta;

    if(turnError > 180 || turnError < -180)
      turnError = turnError - sgn(turnError) * 360;
    
    linearVelocity = linearMTPkP * distance * sgn(cos(turnError * M_PI / 180));
    turnVelocity = turnMTPkP * turnError;

    if(distance < closeTollerance) 
      closeToTarget = true;
    if(closeToTarget) {
      linearVelocity = linearMTPkP * distance * sgn(cos(turnError * M_PI / 180));
      turnError = findMinAngle(targetHeading, globalHeadingInDegrees);
      turnVelocity = turnMTPkP * atan(tan(turnError * M_PI / 180)) * 180 / M_PI;

      setRightPower((linearVelocity + turnVelocity) * 12000 / 127);
      setLeftPower((linearVelocity - turnVelocity) * 12000 / 127);

      if (fabs(sqrt(pow(targetX - globalX, 2) + pow(targetY - globalY, 2))) < closeTollerance) 
        break; 
    }

  }
}

void Drive::movePID(const double inches, const double rpm, const double acceleration, const bool dift, const double secThreshold) {
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

void Drive::turnPID(const double angle, const double rpm, const double acceleration, const double secThreshold) {
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