/**
 * @file drive.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for all of the drive train methods for the
          15in. As of the moment the 15in is a tank drive.
 * @version 0.9
 * @date 2023-04-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "atum8/algorithms/pid.hpp"
#include "atum8/globals.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/misc/task.hpp"
#include "atum8/misc/utility.hpp"
#include "atum8/sensors/imus.hpp"
#include "main.h"

namespace atum8 {
class Drive : Pid, SlewRate, Imus, public Task {
public:
  void taskFn();
  void tankDrive();
  void arcadeDrive();
  void movePID(const double inches, const double rpm, const double acceleration,
               const bool dift, const double secThreshold);
  void turnPID(const double angle, const double rpm, const double acceleration,
               const double secThreshold);
  void moveToPoint(const double desiredX, const double desiredY,
                   const double linearRpm, const double turnRpm,
                   const double acceleration, const double secThreshold);
  void turnToPoint(const double desiredX, const double desiredY,
                   const double rpm, const double acceleration,
                   const double secThreshold);
  void turnToAngle(const double angle, const double rpm,
                   const double acceleration, const double secThreshold);
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

  double aimAssistPower;
  bool isRedAimBotMode{true};
  const short int redID{1};
  const short int blueID{2};
  const short int yellowID{3};
  const short int visionFOVWidth{316};
  const short int visionFOVHeight{212};
};
} // namespace atum8