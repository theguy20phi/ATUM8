#include "atum8\autonRoutes\blueAuton.hpp"
#include "main.h"

namespace atum8 {
void blueAuton() {
  atum8::Catapult catapult;
  atum8::Drive drive;
  atum8::EndGame endGame;
  atum8::Intake intake;
  atum8::Odometry odometry;
  atum8::Vision vision;

      odometry.setStartingPosition(0, 0, -90);

  // Move to first roller
  drive.moveToPoint(20, 0, 100, 100, 3);
  drive.turnToAngle(0, 200, 5);
  drive.moveToPoint(20, -4, 100, 0, 2);

  // Toggle first roller
  intake.setRollerToBlue(2);

  // Face line of 3
   drive.moveToPoint(20, 0, 100, 0, 2);
   drive.turnToAngle(135, 200, 5);

   // Intake line of 3
   intake.in();
   drive.moveToPoint(-37, 46, 100, 0, 5);
   drive.turnToAngle(210, 200, 5);

   // Fire catapult
   catapult.shoot();
   catapult.downUntilPrimed();

   intake.stop();

  //odometry.setStartingPosition(0, 0, -90);

  // Move to first roller
  //drive.moveToPoint(20, 0, 100, 100, 3);
  //drive.turnToAngle(0, 200, 5);
  //drive.moveToPoint(20, -4, 100, 0, 2);

  // Toggle first roller
  //intake.setRollerToBlue(2);

  // Move to mid field
  // drive.moveToPoint(20, 0, 100, 0, 2);
  // drive.turnToAngle(135, 200, 5);

  // // Wait on Braden
  // pros::delay(18000);

  // // Move to second roller
  // intake.in();
  // drive.moveToPoint(-75.75, 86, 100, 100, 5);
  // drive.turnToAngle(80, 200, 5);
  // drive.moveToPoint(-80, 120, 100, 0, 1);

  // Toggle Second roller
  //intake.setRollerToBlue(10);
}
} // namespace atum8