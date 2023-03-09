#pragma once
#include "atum8/globals.hpp"
#include "main.h"

namespace atum8 {

class Catapult {
public:
  void down();
  void up();
  void stop();
  void downUntilPrimed();
  void shoot();
  void controller();

private:
};
} // namespace atum8