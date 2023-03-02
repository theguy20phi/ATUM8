#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8 {
class AutonSelector {
public:
  void selector();

private:
  void scrollRight();
  void scrollLeft();
  void confirmSelection();

  void displayRedSelection();
  void displayRedActive();

  void displayBlueSelection();
  void displayBlueActive();

  void displaySkillsSelection();
  void displaySkillsActive();

  void displayTestingSelection();
  void displayTestingActive();


  int lcdMin{1};
  int lcdMax{4};
  int lcdCurrent{1};
};
} // namespace atum8