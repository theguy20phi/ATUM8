#include "main.h"
#include "atum8/algorithms/odometry.hpp"
#include "atum8/gui/autonSelector.hpp"
#include "atum8/autonRoutes/blueAuton.hpp"
#include "atum8/autonRoutes/programmingSkillsAuton.hpp"
#include "atum8/autonRoutes/redAuton.hpp"
#include "atum8/autonRoutes/testingAuton.hpp"
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
  // Configure LCD Screen
  pros::lcd::initialize();
  pros::lcd::set_background_color(0, 0, 0);
  pros::lcd::set_text_color(255, 255, 255);

  // Configure Optical Sensor
  atum8::opticalSensor.set_led_pwm(100);
  atum8::opticalSensor.disable_gesture();

  // Configure Vision Sensor
  atum8::visionSensorGoal.set_exposure(75);

  pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
  atum8::imuSensorAlpha.reset();
  atum8::imuSensorBeta.reset(true);
 // atum8::imuSensorCharlie.reset(true);

  // Display Autonomous Selector
  pros::lcd::clear_line(1);
  atum8::AutonSelector autonSelector;
  while (pros::competition::is_disabled()) {
    autonSelector.selector();
    pros::delay(10);
  }
}

void disabled() {}

void competition_initialize() {}

void autonomous() { 
  if(atum8::program == 1)
    atum8::redAuton();
  else if(atum8::program == 2)
    atum8::blueAuton();
  else if(atum8::program == 3)
    atum8::programmingSkillsAuton();
  else if(atum8::program == 4)
    atum8::testingAuton();
}

void opcontrol() {
  // Stops the Catapult from shooting right from the start

  pros::delay(200);
  drive.start();
  catapult.start();
  intake.start();
  endgame.start();
}
