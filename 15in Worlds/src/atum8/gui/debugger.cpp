#include "atum8/gui/debugger.hpp"
#include "main.h"

namespace atum8 {
void Debugger::taskFn() {
  while (true) {
    displayCoordinates();
    pros::delay(10);
  }
}

void displayCoordinates() {
  std::string xString = "X: " + std::to_string(globalX) + " Inches";
  std::string yString = "Y: " + std::to_string(globalY) + " Inches";
  std::string globalHeadingInDegreesString = "Heading: " + std::to_string(globalHeadingInDegrees) + " Degrees";

  pros::lcd::set_text(1, xString);
  pros::lcd::set_text(2, yString);
  pros::lcd::set_text(3, globalHeadingInDegreesString);
}

} // namespace atum8