#include "main.h"
#include "okapi/api/units/QLength.hpp"

constexpr okapi::QAngularSpeed lowSpeed{2400_rpm};
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

	auto forwardPidFF = std::make_shared<atum8::Slider>(std::make_unique<atum8::PidFF>(24, 0.01),
														std::make_unique<atum8::PidFF>(7.5, 0.001),
														4,
														0.9);
	auto turnPidFF = std::make_shared<atum8::Slider>(std::make_unique<atum8::PidFF>(10.25, 0.01),
													 std::make_unique<atum8::PidFF>(3, 0.0015),
													 5,
													 0.9);
	drive = atum8::SPMecanumBuilder()
				.withRFMotor(-11)
				.withLFMotor(1)
				.withLBMotor(10)
				.withRBMotor(-20)
				.withImus({14, 15, 18}, 1)
				.withBaseWidth(14.625_in)
				.withWheelCircum(5.4_in)
				.withForwardController(forwardPidFF)
				.withForwardSlew(3)
				.withTurnController(turnPidFF)
				.withTurnSlew(6)
				.withForwardSettledChecker(1_in, 1_inps, 0.375_s)
				.withTurnSettledChecker(1.5_deg, 10_degps, 0.375_s)
				.withStickFunction([](int input)
								   { return pow(input, 3) / 16129; })
				.build();
	drive->reset();
	gui->view();

	auto flywheelVelController = std::make_shared<atum8::PidFF>(0.011, 0, 0, 4.23);
	auto flywheelSettledChecker = std::make_shared<atum8::SettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>>(100_rpm, 0_rpmps, 0_s);
	flywheel = atum8::SPFlywheelBuilder()
				   .withMotors({16, -17})
				   .withController(flywheelVelController)
				   .withSettledChecker(flywheelSettledChecker)
				   .withSpeedMultiplier(15.0)
				   .build();
	flywheel->start();

	intake = atum8::SPIntakeBuilder()
				 .withMotor(13)
				 .withPiston('H')
				 .withFlywheel(flywheel)
				 .withShotDelay(300)
				 .build();
	intake->start();

	roller = atum8::SPRollerBuilder()
				 .withMotor(5)
				 .withOptical(6)
				 .withColor(atum8::Color::Red)
				 .build();
	roller->start();

	endGame = std::make_unique<pros::ADIDigitalOut>('G');
}

void disabled()
{
	gui = autonSelector;
	while (true)
	{
		gui->view();
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
	intake->shoot(0, false);
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
		else if (master.get_digital_new_press(DIGITAL_R1))
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
			readyNotified = false;
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
	roller->setColor(atum8::Color::Red);
	const okapi::QLength offsetDistance{4_in};
	flywheel->setReferenceSpeed(lowSpeed);
	turnRoller();
	drive->forward(offsetDistance, 1_s);
	drive->turn(135_deg, 2_s);
	intake->runIntake();
	drive->forward(-0.9_tile, 2_s, 60);
	drive->turn(-45_deg, 2_s);
	drive->forward(-0.175_tile, 2_s);
	turnRoller();
	intake->stopIntake();
	drive->forward(0.375_tile, 1_s);
	drive->turn(-90_deg, 2_s);
	drive->forward(1.35_tile, 4_s);
	drive->turn(5_deg, 1_s);
	intake->shoot(3);
	atum8::waitFor([]()
				   { return !intake->isShooting(); });
	drive->turn(-5_deg, 1_s);
	drive->forward(-0.7_tile, 3_s);
	drive->turn(-135_deg, 2_s);
	intake->runIntake();
	drive->forward(-2.15_tile, 5_s, 50);
	drive->turn(100_deg, 1_s);
	intake->shoot(3);
	atum8::waitFor([]()
				   { return !intake->isShooting(); });
	drive->turn(-55_deg, 1_s);
	drive->forward(-1_tile, 2_s);
	drive->turn(135_deg, 2_s);
	drive->forward(-1.9_tile, 5_s, 50);
	drive->turn(45_deg, 1_s);
	drive->forward(1_tile, 2_s);
	drive->turn(38_deg, 1_s);
	intake->shoot(3);
	atum8::waitFor([]()
				   { return !intake->isShooting(); });
	intake->stopIntake();
	flywheel->setReferenceSpeed(highSpeed);
	drive->turn(-38_deg, 1_s);
	drive->forward(-1_tile, 2_s);
	drive->turn(-45_deg, 1_s);
	intake->runIntake();
	drive->forward(-1.5_tile, 6_s, 30);
	drive->turn(55_deg, 1_s);
	intake->shoot(3);
	atum8::waitFor([]()
				   { return !intake->isShooting(); });
	drive->turn(-55_deg, 1_s);
	endGame->set_value(1);
}

void match()
{
	drive->forward(4_in);
}

void special()
{
}

void turnRoller()
{
	drive->forward(-20_ft, 0.5_s, 25);
	roller->turnToColor();
	drive->forward(-20_ft, 1.5_s, 25);
}
