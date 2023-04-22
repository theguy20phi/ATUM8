#include "atum8/globals.hpp"
#include "atum8/sensors/vision.hpp"
#include "main.h"

#include "pros/rtos.hpp"
#include "pros/vision.h"

namespace atum8 {

Pid aimBotController(50, 0, 0, 5, .05);
SlewRate slew;
// Drive drive;

void Vision::redAimBot(const double secThreshold) {
  msCounter = 0;
  pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(
      redID, 9127, 10643, 9884, -607, 1, -302, 6.9, 0);
  visionSensorGoal.set_signature(redID, &RED_SIG);
  aimBotController.reset();
  aimBotController.setMaxOutput(6000);

  while (true) {
    pros::vision_object_s_t redGoal = visionSensorGoal.get_by_sig(0, redID);
    double power = aimBotController.getOutput(redGoal.x_middle_coord,
                                              visionFOVWidth * 0.5);//change desired

    if(aimBotController.isSettled())
      break;
    msCounter += 10;
    if(msCounter/1000 > secThreshold)
      break;

    setRightPower(slew.getOutput(getRightPower(), power, 600));
    setLeftPower(slew.getOutput(getLeftPower(), -power, 600));

    pros::delay(10);
  }
}

void Vision::blueAimBot(const double secThreshold) {
  msCounter = 0;
  pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(
      blueID, -3615, -2849, -3232,  7837,  9429,  8634, 3.7, 0);
  visionSensorGoal.set_signature(blueID, &BLUE_SIG);
  
  aimBotController.reset();
  aimBotController.setMaxOutput(6000);
  while (true) {
    pros::vision_object_s_t blueGoal = visionSensorGoal.get_by_sig(0, blueID);
    double power =
        aimBotController.getOutput(blueGoal.x_middle_coord, visionFOVWidth * 0.5);

    if(aimBotController.isSettled())
      break;
    msCounter += 10;
    if(msCounter/1000 > secThreshold)
      break;

    setRightPower(slew.getOutput(getRightPower(), power, 600));
    setLeftPower(slew.getOutput(getLeftPower(), -power, 600));

    pros::delay(10);
  }
}

void Vision::diskAimBot(const double secThreshold) {
  msCounter = 0;
  pros::vision_signature_s_t YELLOW_SIG = pros::Vision::signature_from_utility(
      yellowID, 2159, 3375, 2766, -4481, -4079, -4280, 7.4, 0);
  visionSensorDisk.set_signature(yellowID, &YELLOW_SIG);

  aimBotController.setMaxOutput(3000);

  while (true) {
    pros::vision_object_s_t disk = visionSensorDisk.get_by_sig(0, yellowID);
    double power =
        aimBotController.getOutput(disk.x_middle_coord, visionFOVWidth * 0.5);

    if(aimBotController.isSettled())
      break;
    msCounter += 10;
    if(msCounter/1000 > secThreshold)
      break;

    setRightPower(slew.getOutput(getRightPower(), power, 600));
    setLeftPower(slew.getOutput(getLeftPower(), -power, 600));

    pros::delay(10);
  }
}
} // namespace atum8
