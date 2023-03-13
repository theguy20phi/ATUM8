#include "atum8\autonRoutes\programmingSkillsAuton.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"
#include "main.h"


namespace atum8 {
void programingSkillsAuton() {
  atum8::Drive drive;
  atum8::Catapult catapult;
  atum8::Intake intake;
  atum8::EndGame endGame;

  // Align with roller 1
  drive.move(-25, 200, 1000, false, 1);
  drive.turn(90, 200, 1000, 1);
  drive.move(-7, 200, 1000, false, 1);

  // Toggle roller 1
  intake.setRollerToRed();

  // Align with disk
  drive.move(6, 200, 1000, false, 1);
  drive.turn(-137, 200, 1000, 1);

  // Eat disk
  intake.in();
  drive.move(-23.5, 100, 1000, false, 3);

  // Align with roller 2
  drive.turn(45, 200, 1000, 1);
  pros::delay(500);
  intake.stop();
  drive.move(-7, 200, 1000, false, 1);

  // Toggle roller 2
  intake.setRollerToRed();

  // Align with red goal
  drive.move(5, 200, 1000, false, 1);
  intake.in();
  drive.turn(90, 200, 1000, 1);
  drive.move(57, 200, 1000, false, 2);

  // Adjust to goal
  drive.turn(-9, 200, 2000, .5);

  // Shoot Disks into red goal
  catapult.shoot();
  intake.stop();

  // Drive back towards 3 stack of disks
  drive.turn(11, 200, 2000, .5);
  drive.move(-29, 200, 1000, false, 2);
  drive.turn(92, 200, 2000, 1);

  // Hit wall to readjust robot part 1
  drive.move(10, 50, 1000, false, 1);

  // Eat 3 stack of disks
  intake.in();
  drive.move(-17, 200, 1000, false, 1);
  drive.move(-18, 40, 1000, false, 3);
  pros::delay(500);

  // Hit wall to readjust robot part 2
  drive.move(30, 200, 1000, false, 1);
  drive.move(5, 50, 1000, false, 1);

  // Align with red goal part 2
  drive.move(-4, 200, 1000, false, 1);
  drive.turn(-90, 200, 1000, 1);
  drive.move(30, 200, 1000, false, 2);

  // Adjust to the goal
  drive.turn(-10, 200, 2000, .5);

  // Shoot 3 stack of disks into red goal
  catapult.shoot();
  intake.stop();

  // Drive back toward 3 stack of disks part 2
  drive.turn(12, 200, 2000, .5);
  drive.move(-35, 200, 1000, false, 2);
  drive.turn(90, 200, 1000, 1);

  // Hit wall to readjust part 3
  drive.move(10, 50, 4000, false, 1);

  // Align with 3 stack of disks part 2
  drive.move(-5.8, 200, 1000, false, 1);
  drive.turn(-45, 200, 2000, 1);

  // Eat 3 stack of disks part 2
  intake.in();
  drive.move(-20, 200, 1000, false, 1);
  drive.move(-29.5, 40, 1000, false, 3);

  // Align with blue goal
  drive.move(-14, 200, 1000, false, 1);
  drive.turn(-135, 200, 2000, 1.5);
  drive.move(32, 200, 1000, false, 1);

  // Adjust to goal
  drive.turn(10, 200, 2000, .5);

  // Shoot 3 stack of disks into blue goal
  catapult.shoot();
  intake.stop();

  // Align with row of 3 disks
  drive.turn(-11, 200, 2000, 1);
  drive.move(-53, 200, 1000, false, 2);
  drive.turn(-135, 200, 2000, 2);

  // Eat row of 3 disks
  intake.in();
  drive.move(-20, 200, 1000, false, 1);
  drive.move(-47, 100, 1000, false, 4);

  // Adjust to blue goal part 2
  drive.turn(92, 200, 2000, 2);
  //drive.move(5, 100, 1000, false, 1);


  // Shoot row of 3 disks into blue goal
  catapult.shoot();
  intake.stop();

  // Align bot for endgame
  drive.turn(90, 200, 2000, 1);
  drive.move(-60, 200, 1000, false, 2);
  drive.turn(9, 200, 2000, 1);

  // Wait for Braden's bot
  pros::delay(2000);

  endGame.shoot();
  catapult.down();
}
} // namespace atum8
