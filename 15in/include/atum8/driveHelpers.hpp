#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8 {

class DriveHelpers {
public:
  double rpmToPower(double rpm);
  void setRightPower(double power);
  void setLeftPower(double power);
  void setMoveRight(double power);
  void setMoveLeft(double power);
  void rightAcceleration(double rightTarget, double accelerationStepUp);
  void leftAcceleration(double leftTarget, double accelerationStepUp);
  void setDriveBrakeMode(const std::string brakeMode);
  void resetEncoders();
  double getRightEncoderValues();
  double getLeftEncoderValues();
  double getEncoderAverages();
  void resetImuSensors();
  double getImuSensorAverages();


private:
  // Slew Rate Varibales
  double accelerationStepUp{4000};
  double accelerationStepDown{3000};
  double rightPower{0};
  double leftPower{0};
};
} // namespace atum8