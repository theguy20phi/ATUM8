/**
 * @file drive.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for all of the drive train methods for the
 15in. As of the moment the 15in is a tank drive.
 * @version 0.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "atum8/controllers/pid.hpp"
#include "atum8/sensors/imus.hpp"
#include "atum8/slewRate.hpp"
#include "atum8/task.hpp"
#include "main.h"


namespace atum8 {
class Drive : Pid, SlewRate, Imus, public Task {
public:
  void taskFn();
  void controller();
  void moveToReference(double x, double y, double heading, double rpm, double acceleration, double secThreshold);
  void move(double inches, double rpm, double acceleration, bool dift,
            double secThreshold);
  void turn(double angle, double rpm, double acceleration, double secThreshold);
  double getRightPower();
  double getLeftPower();
  double getRightEncoderValues();
  double getLeftEncoderValues();
  double getEncoderAverages();

protected:
  void setRightPower(double power);
  void setLeftPower(double power);
  void setDriveBrakeMode(const std::string brakeMode);

private:
  void resetEncoders();
  void reset();
  double msCounter;
  double power;
};
} // namespace atum8