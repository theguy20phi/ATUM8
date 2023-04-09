#include "atum8/gui/debugger.hpp"
#include "main.h"

namespace atum8 {
void Debugger::taskFn() {
  while (true) {
    displayCoordinates();
    pros::delay(10);
  }
}

void Debugger::displayCoordinates() {
  std::string xString = "X: " + std::to_string(globalX) + " Inches";
  std::string yString = "Y: " + std::to_string(globalY) + " Inches";
  std::string globalHeadingInDegreesString = "Heading: " + std::to_string(globalHeadingInDegrees) + " Degrees";
  std::string globalLinearPowerString = "Linear Power: " + std::to_string(globalLinearPower) + " W";
  std::string globalTurnPowerString = "Turn Power: " + std::to_string(globalTurnPower) + " W";

  pros::lcd::set_text(1, xString);
  pros::lcd::set_text(2, yString);
  pros::lcd::set_text(3, globalHeadingInDegreesString);
  pros::lcd::set_text(4, globalLinearPowerString);
  pros::lcd::set_text(5, globalTurnPowerString);
}

} // namespace atum8