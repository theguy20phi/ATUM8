#include "atum8\autonRoutes\testingAuton.hpp"
#include "main.h"

namespace atum8 {
void testingAuton() {
  atum8::Catapult catapult;
  atum8::Drive drive;
  atum8::EndGame endGame;
  atum8::Intake intake;
  atum8::Odometry odometry;
  atum8::Vision vision;


    drive.movePID(-21, 200, 600, false, 3);

  pros::delay(250);
  
  drive.turnPID(92, 200, 600, 5);

  drive.movePID(-3, 50, 600, false, 1);

  intake.setRollerToRed(2);

  drive.movePID(3, 200, 600, false, 1);

  intake.in();

  drive.turnPID(150, 200, 600, 3);

  pros::delay(30000);

  drive.movePID(-130, 200, 600, false, 10);

  drive.turnPID(-45, 200, 600, 3);

  intake.setRollerToRed(5);
}
} // namespace atum8