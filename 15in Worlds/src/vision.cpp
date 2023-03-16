#include "atum8/vision.hpp"
#include "atum8/globals.hpp"
#include "atum8/systems/drive.hpp"
#include "main.h"

#include "pros/vision.h"

namespace atum8 {
void Vision::redAimBot() {

  pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(redID, 6749, 8023, 7386, -1085, 221, -432, 2.500, 0);
  visionSensor.set_signature(redID, &RED_SIG);

  while (true) {
    pros::vision_object_s_t redGoal = visionSensor.get_by_sig(0, redID);
    
    std::cout << "Red Goal X Value: " << redGoal.x_middle_coord << std::endl;
    if (redGoal.x_middle_coord < visionFOVWidth / 2 - 5) {
      // Turn Left
      Drive::setRightPower(3000);
      Drive::setLeftPower(-3000);
    } else if (redGoal.x_middle_coord > visionFOVWidth / 2 + 5) {
      // Turn Right
      Drive::setRightPower(-3000);
      Drive::setLeftPower(3000);
    } else {
      Drive::setRightPower(0);
      Drive::setLeftPower(0);
      Drive::setDriveBrakeMode("COAST");
    }
  }
}
} // namespace atum8
