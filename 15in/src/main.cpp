#include "main.h"
#include "atum8\autonSelector.hpp"
#include "atum8\catapult.hpp"
#include "atum8\drive.hpp"
#include "atum8\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\intake.hpp"

#include "atum8\autonRoutes\redAuton.hpp"
#include "atum8\autonRoutes\blueAuton.hpp"
#include "atum8\autonRoutes\programmingSkillsAuton.hpp"
#include "atum8\autonRoutes\testingAuton.hpp"

// hello
void initialize() {
  // Initialize LCD Simulator in DARK MODE!!!
  pros::lcd::initialize();
  pros::lcd::set_background_color(0, 0, 0);
  pros::lcd::set_text_color(255, 255, 255);

  // Enable Optical Sensors
  atum8::opticalSensor.set_led_pwm(100);
  atum8::opticalSensor.disable_gesture();

  // Calibrate Inertial Sensors
  atum8::imuSensorAlpha.reset();
  atum8::imuSensorBeta.reset();
  atum8::imuSensorCharlie.reset();
  while (atum8::imuSensorAlpha.is_calibrating() ||
         atum8::imuSensorBeta.is_calibrating() ||
         atum8::imuSensorCharlie.is_calibrating()) {
    pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
    pros::delay(100);
  }

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
  if (atum8::program == 1) 
    atum8::redAuton();
  else if (atum8::program == 2)
    atum8::blueAuton();
  else if (atum8::program == 3)
    atum8::programingSkillsAuton();
  else if (atum8::program == 4) 
    atum8::testingAuton();
}

  void opcontrol() {
    atum8::Drive drive;
    atum8::Catapult catapult;
    atum8::Intake intake;
    atum8::EndGame endGame;

    // So Catapult Doesn't Fire
    pros::delay(100);

    while (true) {
      drive.controller();
      catapult.controller();
      intake.controller();
      endGame.controller();
      pros::delay(10);
    }
  }
