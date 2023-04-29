#include "atum8\autonRoutes\redAuton.hpp"
#include "main.h"

namespace atum8 {
void redAuton() {
  atum8::Catapult catapult;
  atum8::Drive drive;
  atum8::EndGame endGame;
  atum8::Intake intake;
  atum8::Odometry odometry;
  atum8::Vision vision;
  //atum8::Debugger debugger;
    // Intilitze the Odometry code
  //debugger.start();
  odometry.setStartingPosition(0, 0, -90);

  // Move to first roller
  drive.moveToPoint(20, 0, 100, 100, 3);
  drive.turnToAngle(0, 200, 5);
  drive.moveToPoint(20, -4, 100, 0, 2);

  // Toggle first roller
  intake.setRollerToRed(2);

  // Move to mid field
  drive.moveToPoint(20, 0, 100, 0, 2);
  drive.turnToAngle(135, 200, 5);

  // Wait on Braden
  pros::delay(18000);

  // Move to second roller
  intake.in();
  drive.moveToPoint(-75.75, 86, 100, 100, 5);
  drive.turnToAngle(80, 200, 5);
  drive.moveToPoint(-80, 120, 100, 0, 1);

  // Toggle Second roller
  intake.setRollerToRed(10);
}
} // namespace atum8