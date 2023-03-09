#include "atum8/systems/endGame.hpp"
#include "main.h"

namespace atum8 {

void EndGame::shoot() {
  endGame.set_value(true);
  std::cout << "shooting" << std::endl;
}

void EndGame::retract() {
  endGame.set_value(false);
  std::cout << "retracting" << std::endl;
}

void EndGame::controller() {
  if (Chris.get_digital(DIGITAL_UP))
    shoot();
  else if (Chris.get_digital(DIGITAL_DOWN))
    retract();
}
} // namespace atum8