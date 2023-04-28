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

  drive.movePID(-20, 200, 600, false, 3);
  pros::delay(250);
  drive.turnPID(90, 200, 600, 5);

  drive.movePID(-3, 50, 600, false, 1);

  intake.setRollerToRed(2);

  drive.movePID(3, 200, 600, false, 1);

  drive.turnPID(-93.5, 200, 600, 3);

  drive.movePID(106, 100, 600, false, 8);

  drive.movePID(-3, 200, 600, false, 3);

  drive.turnPID(90, 200, 600, 3);

  drive.movePID(-10, 80, 600, false, 1);

  pros::delay(15000);

  drive.movePID(100, 100, 600, false, 10);

  drive.turnPID(84, 200, 600, 5);

  drive.movePID(-10, 80, 600, false, 3);

  intake.setRollerToRed(5);
}
} // namespace atum8