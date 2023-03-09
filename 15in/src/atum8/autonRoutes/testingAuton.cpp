#include "atum8\autonRoutes\testingAuton.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"
#include "main.h"

namespace atum8 {
void testingAuton() {
  atum8::Drive drive; 
  atum8::Catapult catapult;
  atum8::Intake intake;
  atum8::EndGame endGame;

   intake.setRollerToRed();
   drive.move(5, 200, 1000, false, 1);
   drive.turn(-100, 200, 1000, 2);
   intake.in();
   drive.move(-49, 200, 1000, false, 2);
   drive.move(-7, 50, 1000, false, 1);
   drive.move(10, 200, 1000, false, 1);
}
} // namespace atum8