#include "main.h"
#include "main.h"
//#include "atum8\autonSelector.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\intake.hpp"


void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {

}

void opcontrol() {
		atum8::Drive drive;
	drive.move(24, 200, 600, false, 1);
	drive.move(-10, 200, 600, false, 1);
	drive.move(-5, 200, 600, false, 1);
}
