/**
 * @file endGame.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for all of the end game methods. 
 * @version 0.1
 * @date 2023-03-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "atum8/globals.hpp"
#include "atum8/misc/task.hpp"
#include "main.h"

namespace atum8 {
class EndGame : public Task {
public:
  void taskFn();
  void controller();
  void shoot();
  void retract();
private:
};
} // namespace atum8