/**
 * @file drive.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for all of the drive train methods for the
 15in. As of the moment the 15in is a tank drive.
 * @version 0.6
 * @date 2023-04-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "atum8/algorithms/pid.hpp"
#include "atum8/sensors/imus.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/misc/task.hpp"
#include "atum8/misc/utility.hpp"
#include "main.h"


namespace atum8 {
class Drive : Pid, SlewRate, Imus, public Task{
public:
  void taskFn();
  void tankDrive();
  void arcadeDrive();
  void movePID(const double inches, const double rpm, const double acceleration, const bool dift, const double secThreshold);
  void turnPID(const double angle, const double rpm, const double acceleration, const double secThreshold);
  void moveToPoint(const double desiredX, const double desiredY, const double linearRpm, const double turnRpm, const double secThreshold);
  void turnToPoint(const double desiredX, const double desiredY, const double rpm, const double secThreshold);
  void turnToAngle(const double angle, const double rpm, const double secThreshold);
  double getRightPower();
  double getLeftPower();
  double getEncoderAverages();

protected:
  void setRightPower(double power);
  void setLeftPower(double power);
  void setDriveBrakeMode(const std::string brakeMode);

private:
  double getRightEncoderValues();
  double getLeftEncoderValues();
  void resetEncoders();
  void reset();
  double msCounter;
  double output;

  double linearMaxPower;
  double turnMaxPower;
  double errorX;
  double errorY;
  double linearError;
  double turnError;
  double linearOutput;
  double turnOutput;
  double headingScaleFactor;
};
} // namespace atum8