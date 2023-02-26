#include "main.h"
#include "okapi/api/units/QLength.hpp"

constexpr okapi::QAngularSpeed lowSpeed{2100_rpm};
constexpr okapi::QAngularSpeed highSpeed{2800_rpm};

void initialize()
{
	initializeLCD();
	autonSelector = std::make_shared<atum8::AutonSelector>();
	debugger = std::make_shared<atum8::Debugger>(atum8::Debugger::LineFns{
		[](int control)
		{ return "HELLO WORLD!"; },
		[](int control)
		{ return "IT'S YA BOY:"; },
		[](int control)
		{ return "ANUBIS"; }});
	gui = autonSelector;

	auto forwardPidFF = std::make_shared<atum8::PidFF>(6.0, 0.2);
	auto turnPidFF = std::make_shared<atum8::PidFF>(3.0, 0.1);
	drive = atum8::SPMecanumBuilder()
				.withRFMotor(-10)
				.withLFMotor(20)
				.withLBMotor(11)
				.withRBMotor(-1)
				.withImus({19}, 1.0)
				.withBaseWidth(12.75_in)
				.withWheelCircum(6.28_in)
				.withForwardController(forwardPidFF)
				.withForwardSlew(2.5)
				.withTurnController(turnPidFF)
				.withForwardSettledChecker(2_in, 1_inps, 0.5_s)
				.withTurnSettledChecker(2_deg, 10_degps, 0.5_s)
				.withStickFunction([](int input)
								   { return pow(input, 3) / 16129; })
				.build();
	drive->reset();

	auto flywheelVelController = std::make_shared<atum8::PidFF>(0.01, 0, 0, 4.23);
	auto flywheelSettledChecker = std::make_shared<atum8::SettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>>(150_rpm, 0_rpmps, 0_s);
	flywheel = atum8::SPFlywheelBuilder()
				   .withMotors({16, -17})
				   .withController(flywheelVelController)
				   .withSettledChecker(flywheelSettledChecker)
				   .withSpeedMultiplier(15.0)
				   .build();
	flywheel->start();

	intake = atum8::SPIntakeBuilder()
				 .withMotor(13)
				 .withPiston('A')
				 .withFlywheel(flywheel)
				 .build();
	intake->start();

	roller = atum8::SPRollerBuilder()
				 .withMotor(5)
				 .withOptical(6)
				 .withColor(atum8::Color::Red)
				 .build();
	roller->start();

	endGame = std::make_unique<pros::ADIDigitalOut>('B');
}

void disabled()
{
	gui = autonSelector;
	gui->view();
	while (true)
	{
		roller->setColor(autonSelector->getMatchInfo().color);
		pros::delay(10);
	}
}

void autonomous()
{
	switch (autonSelector->getMatchInfo().routine)
	{
	case atum8::Routine::Skills:
		skills();
		break;
	case atum8::Routine::Match:
		match();
		break;
	case atum8::Routine::Special:
		special();
		break;
	default:
		break;
	}
}

void opcontrol()
{
	gui = debugger;
	gui->view();
	pros::Controller master{pros::controller_id_e_t::E_CONTROLLER_MASTER};
	flywheel->setReferenceSpeed(lowSpeed);
	bool manualRoller{true};
	bool readyNotified{false};

	while (true)
	{
		// Drive Controls
		const int forward{master.get_analog(ANALOG_LEFT_Y)};
		const int strafe{master.get_analog(ANALOG_LEFT_X)};
		const int turn{master.get_analog(ANALOG_RIGHT_X)};
		drive->driver(forward, strafe, turn);

		if (master.get_digital_new_press(DIGITAL_B))
		{
			if (drive->getDriverSettings()->brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST)
				drive->getDriverSettings()->brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
			else
				drive->getDriverSettings()->brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
		}

		if (master.get_digital_new_press(DIGITAL_A))
			drive->getDriverSettings()->maxPower = drive->getDriverSettings()->maxPower == 1.0 ? 0.5 : 1.0;

		// Intake Controls
		if (master.get_digital(DIGITAL_L1))
			intake->runIntake(127);
		else if (master.get_digital(DIGITAL_L2))
			intake->runIntake(-127);
		else
			intake->runIntake(0);

		// Roller Controls
		if (manualRoller)
		{
			if (master.get_digital(DIGITAL_R1))
				roller->runRoller();
			else
				roller->stopRoller();
		}
		else
			roller->turnToColor();

		if (master.get_digital_new_press(DIGITAL_X))
			manualRoller = !manualRoller;

		// Shooter Controls
		if (master.get_digital_new_press(DIGITAL_R2))
			intake->shoot(1, false);

		if (intake->isShooting())
			readyNotified = false;
		else if (flywheel->readyToFire() && !readyNotified)
		{
			master.rumble("..");
			readyNotified = true;
		}

		if (master.get_digital_new_press(DIGITAL_Y))
		{
			switch ((int)flywheel->getReferenceSpeed().convert(okapi::rpm))
			{
			case 0:
				flywheel->setReferenceSpeed(lowSpeed);
				break;
			case (int)lowSpeed.convert(okapi::rpm):
				flywheel->setReferenceSpeed(highSpeed);
				break;
			default:
				flywheel->setReferenceSpeed(0_rpm);
				break;
			}
		}

		// End Game Controls
		if (master.get_digital(DIGITAL_UP) &&
			master.get_digital(DIGITAL_DOWN) &&
			master.get_digital(DIGITAL_LEFT) &&
			master.get_digital(DIGITAL_RIGHT))
			endGame->set_value(1);

		pros::delay(atum8::stdDelay);
	}
}

/* -------------------------------------------------------------------------- */
/*                                   Helpers                                  */
/* -------------------------------------------------------------------------- */

void initializeLCD()
{
	pros::lcd::initialize();
	pros::lcd::set_background_color(lv_color_t{0x000000});
	pros::lcd::set_text_color(255, 255, 255);
	pros::lcd::register_btn0_cb([]()
								{ gui->control(-1); });
	pros::lcd::register_btn1_cb([]()
								{ gui->control(0); });
	pros::lcd::register_btn2_cb([]()
								{ gui->control(1); });
}

void skills()
{
	const okapi::QLength offsetDistance{4_in};
	flywheel->setReferenceSpeed(lowSpeed);
	turnRoller();
	drive->forward(offsetDistance, 0.5_s);
	drive->turn(170_deg, 1_s);
	intake->runIntake();
	drive->forward(-1_tile, 1_s);
	drive->turn(-80_deg, 0.5_s);
	drive->forward(-0.5_tile, 0.5_s);
	intake->stopIntake();
	turnRoller();
	drive->forward(0.5_tile, 0.5_s);
	drive->turn(-90_deg, 1_s);
	drive->forward(2_tile, 2_s);
	drive->turn(-2_deg, 0.25_s);
	intake->shoot(3);
	drive->turn(2_deg, 0.25_s);
	drive->forward(-1.25_tile, 1.5_s);
	drive->turn(-45_deg, 0.5_s);
	intake->runIntake();
	drive->forward(-2.12_tile, 5_s, 40);
	drive->turn(-135_deg, 1.5_s);
	intake->stopIntake();
	drive->forward(3_tile, 1_s);
	drive->forward(-1 * offsetDistance, 0.5_s);
	drive->turn(-90_deg, 1_s);
	drive->forward(1_tile, 1_s);
	intake->shoot(3);
	drive->forward(-1_tile, 1_s);
	drive->turn(90_deg, 1_s);
	drive->forward(3_tile, 1_s);
	drive->forward(-0.25_tile - offsetDistance, 0.5_s);
	intake->runIntake();
	drive->forward(-0.75_tile, 5_s, 40);
	drive->forward(3_tile, 1_s);
	intake->stopIntake();
	drive->forward(-1 * offsetDistance, 0.5_s);
	drive->turn(-90_deg, 1_s);
	drive->forward(1_tile, 1_s);
	intake->shoot(3);
	drive->forward(-0.5_tile, 0.5_s);
	drive->turn(-90_deg, 1_s);
	drive->forward(3_tile, 1_s);
	drive->forward(-1.5_tile - offsetDistance, 2_s);
	drive->turn(45_deg, 0.5_s);
	intake->runIntake();
	drive->forward(-2.12_tile, 2_s);
	drive->turn(-90_deg, 1_s);
	intake->stopIntake();
	drive->forward(1.41_tile, 2_s);
	drive->turn(45_deg, 0.5_s);
	intake->shoot(3);
	drive->forward(-0.5_tile, 0.5_s);
	drive->turn(-90_deg, 1_s);
	drive->forward(3_tile, 1_s);
	drive->forward(-1 * offsetDistance, 0.5_s);
	drive->turn(180_deg, 2_s);
	drive->forward(4.5_tile, 5_s);
	drive->turn(90_deg, 1_s);
	drive->forward(-2_tile, 2_s);
	drive->turn(45_deg, 0.5_s);
	endGame->set_value(1);
}

void match()
{
}

void special()
{
	flywheel->setReferenceSpeed(lowSpeed);
	drive->forward(-5_ft, 3_s, 15);
	roller->runForAt(320, 127);
	drive->forward(-5_ft, 3_s, 0);
	drive->forward(1.5_ft, 3_s);
	drive->turn(90_deg, 3_s);
	intake->runIntake();
	drive->forward(-1.1_tile, 3_s);
	intake->runIntake(0);
	drive->forward(-5_ft, 3_s, 15);
	roller->runForAt(320, 127);
	drive->forward(-5_ft, 3_s, 0);
	drive->forward(1_ft, 3_s);
	drive->turn(-90_deg, 3_s);
	drive->forward(2_tile, 6_s);
	drive->turn(-2_deg, 3_s);
	intake->shoot(3);
	drive->forward(-0.5_tile, 3_s);
}

void turnRoller()
{
	drive->forward(-20_ft, 0.5_s, 15);
	roller->turnToColor();
	drive->forward(-20_ft, 1.5_s, 15);
}
