#include "main.h"
#include "atum8/sensors/vision.hpp"
#include "main.h"
//#include "atum8\autonSelector.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"
#include "pros/colors.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"


void initialize() {
	  //pros::lcd::initialize();
  //pros::lcd::set_background_color(0, 0, 0);
  //pros::lcd::set_text_color(255, 255, 255);

  // Enable Optical Sensors
  //atum8::opticalSensor.set_led_pwm(100);
  //atum8::opticalSensor.disable_gesture();

  // Calibrate Inertial Sensors
  //atum8::imuSensorAlpha.reset();
  //atum8::imuSensorBeta.reset();
  //atum8::imuSensorCharlie.reset();
  // while (atum8::imuSensorAlpha.is_calibrating() ||
  //        atum8::imuSensorBeta.is_calibrating() ||
  //        atum8::imuSensorCharlie.is_calibrating()) {
  //   pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
  //   pros::delay(100);
  // }
}

void disabled() {}

void competition_initialize() {
}

void autonomous() {

}

void opcontrol() {
		atum8::Drive drive;
    atum8::Vision vision;


    while (true) {
        pros::screen::set_pen(COLOR_RED);
        pros::screen::fill_rect(0,0,240,120);

        pros::screen::set_pen(COLOR_BLUE);
        pros::screen::fill_rect(241,0,480,120);


        pros::screen::set_pen(COLOR_PURPLE);
        pros::screen::fill_rect(0,121,240,240);

        pros::screen::set_pen(COLOR_DARK_ORANGE);
        pros::screen::fill_rect(241,121,480,240);
        
        pros::screen::set_pen(COLOR_GREEN);

        pros::screen_touch_status_s_t status = pros::screen::touch_status();

        if(status.x >= 0 && status.y >= 0 && status.x <= 240 && status.y <= 120) {
          std::cout << "Red" << std::endl;
          pros::screen::fill_rect(0 + 50,0 + 40,240 - 50,120 - 40);
        }
        else if(status.x >= 241 && status.y >= 0 && status.x <= 480 && status.y <= 120) {
          std::cout << "Blue" << std::endl;
          pros::screen::fill_rect(241 + 50,0 + 40,480 - 50,120 - 40);
        }
        else if(status.x >= 0 && status.y >= 121 && status.x <= 240 && status.y <= 240) {
          std::cout << "Purple" << std::endl;
          pros::screen::fill_rect(0 + 50,121 + 40,240 - 50,240 - 40);
        }
        else if(status.x >= 241 && status.y >= 121 && status.x <= 480 && status.y <= 240) {
          std::cout << "Orange" << std::endl;
          pros::screen::fill_rect(241 + 50,121 + 40,480 - 50,240 - 40);
        }
 

pros::delay(100);

    }


    //vision.redAimBot();

	//drive.move(24, 200, 600, false, 1);
	//drive.move(-10, 200, 600, false, 1);
	//drive.move(-5, 200, 600, false, 1);
	//drive.turn(90, 200, 600, 1);
}
