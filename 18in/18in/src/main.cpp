#include "main.h"
#include "okapi/api/units/QLength.hpp"

atum8::SPPidFF forwardPidFF;
atum8::SPPidFF turnPidFF;
atum8::SPPidFF flywheelVelController;
atum8::SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> flywheelSettledChecker;
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

	forwardPidFF = std::make_shared<atum8::PidFF>(6.0, 0.2);
	turnPidFF = std::make_shared<atum8::PidFF>(3.0, 0.1);
	drive = atum8::SPMecanumBuilder()
				.withRFMotor(-10)
				.withLFMotor(20)
				.withLBMotor(11)
				.withRBMotor(-1)
				.withImu(19, 1.0)
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

	intake = atum8::SPIntakeBuilder()
				 .withMotor(13)
				 .withPiston('D')
				 .withLineTrackers('C', 'B', 'A')
				 .withLineTrackerThreshold(1500)
				 .build();
	intake->start();

	roller = atum8::SPRollerBuilder()
				 .withMotor(5)
				 .withOptical(6)
				 .withColor(atum8::Color::Red)
				 .build();
	roller->start();

	flywheelVelController = std::make_shared<atum8::PidFF>(0.01, 0, 0, 4.23);
	flywheelSettledChecker = std::make_shared<atum8::SettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>>(150_rpm, 0_rpmps, 0_s);
	flywheel = atum8::SPFlywheelBuilder()
				   .withMotors({16, -17})
				   .withController(flywheelVelController)
				   .withSettledChecker(flywheelSettledChecker)
				   .withSpeedMultiplier(15.0)
				   .build();
	flywheel->start();
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
	for (int i = 0; i < 3; i++)
	{
		while (!flywheel->readyToFire())
			pros::delay(500);
		intake->shoot();
		pros::delay(500);
	}
	drive->forward(-0.5_tile, 3_s);
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
				roller->runRoller(0);
		}
		else
			roller->turnToColor();

		if(master.get_digital_new_press(DIGITAL_X))
			manualRoller = !manualRoller;


		// Shooter Controls
		if (master.get_digital_new_press(DIGITAL_R2))
			intake->shoot();

		if (intake->isShooting())
			readyNotified = false;
		else if (flywheelSettledChecker->isSettled() && !readyNotified)
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
