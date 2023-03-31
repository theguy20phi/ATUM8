/**
 * @file  autonSelector.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for the autonomous selector on the V5 Brain.
 * @version 0.2
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "atum8/globals.hpp"
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


  short const int lcdMin{1};
  short const int lcdMax{4};
  short int lcdCurrent{1};
};
} // namespace atum8