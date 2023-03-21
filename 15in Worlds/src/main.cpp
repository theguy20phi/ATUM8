#include "main.h"
#include "atum8/vision.hpp"
#include "main.h"
//#include "atum8\autonSelector.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"


void initialize() {
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
}

void disabled() {}

void competition_initialize() {
}

void autonomous() {

}

void opcontrol() {
		atum8::Drive drive;
    atum8::Vision vision;

    vision.redAimBot();

	//drive.move(24, 200, 600, false, 1);
	//drive.move(-10, 200, 600, false, 1);
	//drive.move(-5, 200, 600, false, 1);
	//drive.turn(90, 200, 600, 1);
}
