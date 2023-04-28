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
  if(Chris.get_digital_new_press(DIGITAL_LEFT)) {
    isRightRetracted = !isRightRetracted;
    endGameRight.set_value(!isRightRetracted);
    std::cout << "rightEndGame: " << isRightRetracted <<  std::endl;
  }

  if(Chris.get_digital_new_press(DIGITAL_RIGHT)) {
    isLeftRetracted = !isLeftRetracted;
    endGameLeft.set_value(!isLeftRetracted);
    std::cout << "leftEndGame: " << isLeftRetracted << std::endl;
  }
}


void EndGame::shoot() {
  endGameRight.set_value(true);
  endGameLeft.set_value(true);
  std::cout << "shooting" << std::endl;
}

void EndGame::retract() { 
  endGameRight.set_value(false);
  endGameLeft.set_value(false);
  std::cout << "retracting" << std::endl;
}
} // namespace atum8