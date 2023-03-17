#include "atum8/vision.hpp"
#include "atum8/globals.hpp"
#include "atum8/systems/drive.hpp"
#include "main.h"

#include "pros/vision.h"

namespace atum8 {


Pid aimBotController(0, 0, 0, .5, .05);

void Vision::redAimBot() {
  pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(
      redID, 9127, 10643, 9884, -607, 1, -302, 6.9, 0);
  pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(
      blueID, -3615, -2849, -3232,  7837,  9429,  8634, 3.7, 0);
  pros::vision_signature_s_t YELLOW_SIG = pros::Vision::signature_from_utility(
      yellowID, 2159,  3375,  2766, -4481, -4079, -4280, 7.4, 0);
  visionSensor.set_signature(redID, &RED_SIG);
  visionSensor.set_signature(blueID, &BLUE_SIG);
  visionSensor.set_signature(yellowID, &YELLOW_SIG);

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
