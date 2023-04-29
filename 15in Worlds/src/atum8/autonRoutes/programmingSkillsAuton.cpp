#include "atum8\autonRoutes\programmingSkillsAuton.hpp"
#include "main.h"

namespace atum8 {
void programmingSkillsAuton() {
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
  intake.setRollerToRed(2);

  // Move to mid field
  drive.moveToPoint(20, 0, 100, 0, 2);
  drive.turnToAngle(135, 200, 5);

  // Wait on Braden
  pros::delay(30000);

  endGame.shoot();
}
} // namespace atum8