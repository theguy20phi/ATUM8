#include "main.h"
#include "atum8/algorithms/odometry.hpp"
#include "atum8/sensors/imus.hpp"
#include "atum8/sensors/vision.hpp"
#include "atum8/gui/autonSelector.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"

atum8::Drive drive;
atum8::Intake intake;
atum8::Odometry odometry;
atum8::Imus imus;

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_background_color(0, 0, 0);
  pros::lcd::set_text_color(255, 255, 255);
  imus.calibrateImuSensors();
  atum8::opticalSensor.set_led_pwm(100);
  atum8::opticalSensor.disable_gesture();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
}

void opcontrol() {
  odometry.setStartingPosition(0, 0, 0);
  odometry.start();
}
