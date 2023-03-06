#include "atum8\autonRoutes\blueAuton.hpp"
#include "atum8\catapult.hpp"
#include "atum8\drive.hpp"
#include "atum8\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\intake.hpp"
#include "main.h"

namespace atum8 {
void blueAuton() {
  atum8::Drive drive;
  atum8::Catapult catapult;
  atum8::Intake intake;
  atum8::EndGame endGame;

  drive.move(-5, 200, 1000, false, 1);
   intake.setRollerToBlue();
   drive.move(5, 200, 1000, false, 1);
   drive.turn(-100, 200, 1000, 2);
   intake.in();
   drive.move(-49, 200, 1000, false, 2);
   drive.move(-7, 50, 1000, false, 1);
   drive.move(10, 200, 1000, false, 1);

   //drive.move(5, 200, 1000, false, 1);
   //drive.turn(88, 200, 2000, 1);
   //drive.move(100, 200, 1000, false, 3);

   //drive.move(10, 100, 1000, false, 1);
   //drive.move(-10, 200, 100, false, 1);
/*
  // Align with roller 1
  drive.move(-25, 200, 1000, false, 1);
  drive.turn(90, 200, 1000, 1);
  drive.move(-4.7, 200, 1000, false, 1);

  // Toggle roller 1
  intake.setRollerToBlue();
  //intake.setRollerToBlue();
  //intake.setRollerToRed();

  drive.move(5.5, 200, 1000, false, 1);
  drive.turn(-90, 200, 2000, 1);
  drive.move(99, 200, 1000, false, 3);

  drive.move(10, 50, 1000, false, 1);
  drive.move(-6, 100, 1000, false, 2);

  drive.turn(92.5, 200, 2000, 1);

  drive.move(100, 100, 1000, false, 5);
  drive.turn(90, 200, 2000, 1);

  drive.move(-7, 200, 1000, false, 2);

  intake.setRollerToBlue();

  drive.move(5.5, 200, 1000, false, 1);
  drive.turn(-99, 200, 2000, 1);
  intake.in();
  drive.move(-50, 200, 1000, false, 2);
  drive.move(-6, 50, 1000, false, 2);

  */
}
} // namespace atum8