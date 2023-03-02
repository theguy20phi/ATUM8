#pragma once
#include "globals.hpp"
#include "driveHelpers.hpp"
#include "main.h"

namespace atum8 {

class Intake : DriveHelpers{
public:
  void setIntakePower(double power);
  void in();
  void out();
  void stop();
  void setRollerToRed();
  void setRollerToBlue();
  void controller();
// 90 blue 37 red 
// 85 blue 45 red
// 86 blue 43 red
// 96 blue 35 red
private:
  const float redRollerHue { 42.50 };
  const float blueRollerHue { 89.25 };
  const float rollerColorThreshold { 10 };

};
} // namespace atum8