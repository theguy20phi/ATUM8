#include "main.h"
#include "atum8\autonSelector.hpp"
#include "atum8\catapult.hpp"
#include "atum8\drive.hpp"
#include "atum8\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\intake.hpp"


void initialize() {  
}

void disabled() {
  pros::lcd::clear_line(1);
  pros::lcd::set_text(1, "darn i'm disabled...");
}

// Only runs in a competition switch is plugged in
void competition_initialize(){
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
  while(atum8::imuSensorAlpha.is_calibrating() || atum8::imuSensorBeta.is_calibrating() || atum8::imuSensorCharlie.is_calibrating()) {
   pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
   pros::delay(100);
  }

  // Display Autonomous Selector
  pros::lcd::clear_line(1);
  atum8::AutonSelector autonSelector;
  while(pros::competition::is_disabled()) {
    autonSelector.selector();
    pros::delay(10);
  }
}

  void autonomous() {
    atum8::Drive drive;
    atum8::Catapult catapult;
    atum8::Intake intake;
    atum8::EndGame endGame;
  

   
   
    // Align with roller 1
    drive.move(-23.2, 200, 1000, false, 1);
    drive.turn(89, 200, 1000, 1);

    
    drive.move(-4.7, 200, 1000, false, .5);

    // Toggle roller 1
    intake.setRollerToRed(); 

    // Align with disk
    drive.move(7, 200, 1000, false, 1);
    drive.turn(-137.5, 200, 1000, 1);

    // Eat disk 
    intake.in();
    drive.move(-21.5, 100, 1000, false, 3);

    // Align with roller 2
    drive.turn(45.3, 200, 1000, .8);
    intake.stop();
    
    drive.move(-7, 200, 1000, false, .8);

    // Toggle roller 2
    intake.setRollerToRed();

    // Align with red goal
    drive.move(3.6, 200, 1000, false, 1);
    intake.in();
    drive.turn(91, 200, 1000, 1);
    
    drive.move(66, 200, 1000, false, 1.75);

    // Adjust to red goal
    drive.turn(-17, 200, 1000, .75);

    // Shoot disks into red goal
    catapult.shoot();
    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

    // Align with 3 stack of disks
    drive.turn(20, 200, 1000, .75);

    
    drive.move(-39, 200, 1000, false, 1.5);

    
    drive.turn(85, 200, 1000, .75);

    // Eat 3 stack of disks
    drive.move(-9, 200, 1000, false, 1);
    intake.in();
    drive.move(-24, 40, 1000, false, 3);
/*

    // Align with red goal but now part 2
    drive.move(29, 200, 1000, false, 1.25);
    /*
    drive.turn(-81, 200, 1000, .75);
    drive.move(33, 200, 1000, false, 1);
    intake.stop();

    // Adjust to red goal but now part 2
    drive.turn(-22.9, 200, 2000, 1);

    // Shoot disks into red goal but now part 2
    catapult.shoot();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Align with 3 stack of disks but now part 2
    drive.turn(22, 200, 2000, 1);
    drive.move(-42.8, 200, 1000, false, 1.5);
    drive.turn(43.5, 200, 2000, 1);

    // Eat 3 stack of disks but now part 2
    intake.in();
    drive.move(-11.5, 200, 1000, false, 1);
    drive.move(-20, 40, 1000, false, 3);

    // ALign with blue goal
    drive.move(-28, 2000, 1000, false, 1.25);
    drive.turn(-134, 200, 2000, 1);
    drive.move(31, 200, 1000, false, 1);
    drive.turn(20, 200, 2000, .75);
    
    // shoot disk into blue goal
    catapult.shoot();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Align with 3 stack of disks but now part 3
    drive.turn(-20, 200, 2000, .75);
    drive.move(-51, 200, 1000, false, 2);
    drive.turn(-130, 200, 2000, 1);
    

    */

    if(atum8::program == 1) {
      
    }

    if(atum8::program == 2) {
      intake.setRollerToBlue();
      drive.move(8, 200, 1000, false, 1);
      drive.turn(90, 200, 1000, 1);
      drive.move(60, 200, 1000, false, 3);
      drive.move(-90, 200, 1000, false, 1);
      drive.move(60, 200, 1000, false, 3);
      drive.move(-90, 200, 1000, false, 1);
      intake.setRollerToBlue();  
    }

    if(atum8::program == 4) {
      //intake.setRollerToRed();
    }
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
