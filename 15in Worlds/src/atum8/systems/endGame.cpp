#include "atum8/systems/endGame.hpp"
#include "main.h"
#include "pros/misc.h"

namespace atum8 {
void EndGame::taskFn() {
  while(true) {
    controller();
    pros::delay(10);
  }
}
void EndGame::controller() { 
  if(Chris.get_digital_new_press(DIGITAL_UP)) {
    isRetracted = !isRetracted;
    endGame.set_value(isRetracted);
  }
}

void EndGame::shoot() {
  endGame.set_value(true);
  std::cout << "shooting" << std::endl;
}

void EndGame::retract() {
  endGame.set_value(false);
  std::cout << "retracting" << std::endl;
}
} // namespace atum8