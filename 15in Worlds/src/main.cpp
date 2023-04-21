#include "main.h"
#include "atum8/algorithms/odometry.hpp"
#include "atum8/gui/autonSelector.hpp"
#include "atum8/gui/debugger.hpp"
#include "atum8/sensors/imus.hpp"
#include "atum8/sensors/vision.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\systems\intake.hpp"


atum8::Drive drive;
atum8::Catapult catapult;
atum8::Intake intake;
atum8::EndGame endgame;
atum8::Odometry odometry;
atum8::Imus imus;
atum8::Debugger debugger;
atum8::Vision vision;

void initialize() {
  odometry.setStartingPosition(0, 0, 0);

  // Configure LCD Screen
  pros::lcd::initialize();
  pros::lcd::set_background_color(0, 0, 0);
  pros::lcd::set_text_color(255, 255, 255);

  // Configure Optical Sensor
  atum8::opticalSensor.set_led_pwm(100);
  atum8::opticalSensor.disable_gesture();

  // Configure Vision Sensor
  atum8::visionSensorGoal.set_exposure(75);

  // Display Autonomous Selector
  //pros::lcd::clear_line(1);
  //atum8::AutonSelector autonSelector;
  //while (pros::competition::is_disabled()) {
   // autonSelector.selector();
   // pros::delay(10);
 // }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  odometry.start();
  debugger.start();
  // intake.setRollerToRed();

  // drive.moveToPoint(24, 24, 200, 200, 3);
  // drive.turnToPoint(0, 0, 200, 3);
  // drive.turnToAngle(180, 200, 2);
}

void opcontrol() {
  pros::delay(200);
  drive.start();
  catapult.start();
  intake.start();
  endgame.start();
}
