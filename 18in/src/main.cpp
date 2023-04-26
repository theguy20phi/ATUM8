#include "main.h"

auto doneShooting = []()
{ return !shooter->isShooting(); };

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
		{ return "ANUBIS"; },
		[](int control)
		{
			return "X: " + std::to_string(odometry->getPosition().x.convert(okapi::inch)) + "in";
		},
		[](int control)
		{
			return "Y: " + std::to_string(odometry->getPosition().y.convert(okapi::inch)) + "in";
		},
		[](int control)
		{
			return "H: " + std::to_string(odometry->getPosition().h.convert(okapi::degree)) + "deg";
		},
		[](int control)
		{
			return "Flywheel Speed: " + std::to_string(shooter->getSpeed().convert(okapi::rpm)) + " RPM";
		},
		[](int control)
		{
			return "# of Disks: " + std::to_string(shooter->getDisks());
		}});
	gui = autonSelector;

	odometry = atum8::SPOdometryBuilder()
				   .withLeft('C', 'D')
				   .withSide('E', 'F', 6.25_in, true)
				   .withRight('G', 'H', true)
				   .withEncoderMultiplier(1.0 / 1024 / 4)
				   .withWidth(8.46886927543_in)
				   .withWheelCircum(203.724231788_mm)
				   .withImus({14, 15, 16}, 0.99)
				   .build();
	odometry->start();

	auto forwardPidFF = std::make_shared<atum8::Slider>(std::make_unique<atum8::PidFF>(21, 0.01),
														std::make_unique<atum8::PidFF>(6.5),
														4,
														0.9);

	auto turnPidFF = std::make_shared<atum8::Slider>(std::make_unique<atum8::PidFF>(10, 0.01),
													 std::make_unique<atum8::PidFF>(2.75),
													 5,
													 0.9);

	auto aimPidFF = std::make_shared<atum8::PidFF>(0.9, 0.05, 13);

	auto aimFilter = std::make_shared<atum8::RollingMedian>(5);

	drive = atum8::SPDriveBuilder()
				.withLeftPorts({-6, 7, -8})
				.withRightPorts({-18, 19, 20})
				.withPoseEstimator(odometry)
				.withVision(12)
				.withStickFunction([](int input)
								   { return pow(input, 3) / 16129; })
				.withAutonSelector(autonSelector)
				.withLateralController(forwardPidFF)
				.withAngularController(turnPidFF)
				.withAimController(aimPidFF)
				.withLateralSettledChecker(1_in, 1_inps, 0.3_s)
				.withAngularSettledChecker(1.0_deg, 50_degps, 0.3_s)
				.withAimFilter(aimFilter)
				.build();

	auto flywheelVelController = std::make_shared<atum8::PidFF>(0.0035, 0, 0, .0666666);
	auto flywheelVelFilter = std::make_shared<atum8::RollingAverage>(30);
	shooter = atum8::SPShooterBuilder()
				  .withFlywheelMotors({9, 10})
				  .withIndexerMotor(2)
				  .withAngleAdjuster(4, 'A')
				  .withIntakeMotor(1)
				  .withIntakeAdjuster(4, 'B')
				  .withLoader(4, 'C')
				  .withPotentiometer('A', {3300, 3190, 2975, 2815, 2420})
				  .withGearing(15.0)
				  .withController(flywheelVelController)
				  .withSettledChecker(15_rpm, 45_rpmps)
				  .withMultiShotAdjustment(500_rpm)
				  .withFilter(flywheelVelFilter)
				  .build();
	shooter->start();

	roller = atum8::SPRollerBuilder()
				 .withMotor(-3)
				 .withOpticals(5, 13)
				 .withAutonSelector(autonSelector)
				 .withRedHue(110)
				 .withBlueHue(20)
				 .build();
	roller->start();

	endGame = std::make_unique<pros::ADIDigitalOut>(pros::ext_adi_port_pair_t{4, 'D'});

	gui->view();
}

void disabled()
{
	gui = autonSelector;
	while (true)
	{
		gui->view();
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
	}
}

void opcontrol()
{
	gui = debugger;
	gui->view();
	pros::Controller master{pros::controller_id_e_t::E_CONTROLLER_MASTER};
	while (true)
	{
		gui->view();
		drive->control(master);
		shooter->control(master);
		roller->control(master);
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
			endGame->set_value(1);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			autonSelector->setColor(autonSelector->getColor() == atum8::Color::Red ? atum8::Color::Blue : atum8::Color::Red);
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
	pros::lcd::print(0, "Don't touch!");
	pros::lcd::print(1, "IMUs are calibrating...");
}

void skills()
{
	odometry->setPosition({0_tile, 0_tile, 0_deg}); // MEASURE ME!
	shooter->singleShotPrepare(2565_rpm);			// Good starting value, adjust
	shooter->raiseLoader();
	pros::delay(2000);
	shooter->singleShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	for (int i{0}; i < 2; i++)
	{
		shooter->raiseLoader();
		pros::delay(5000);
		shooter->singleShot(shooter->getDisks());
		atum8::waitFor(doneShooting);
	}
	shooter->multiShotPrepare(2850_rpm);
	drive->moveTo({2.5_tile, 0_tile}, 0_s, true);
	shooter->raiseIntake();
	drive->moveTo({1.5_tile, -0.5_tile}, 0_s, false, 80, 127, diskOffset);
	shooter->runIntake();
	drive->moveTo({2.5_tile, 0_tile}, 0_s, true);
	drive->moveTo({2.5_tile, 0.5_tile});
	drive->pointAt(goal, 0_s, false, true);
	shooter->multiShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	drive->moveTo({2.5_tile, 0_tile}, 0_s, true);
	shooter->raiseIntake();
	drive->moveTo({1.5_tile, -1.5_tile}, 0_s, false, 80, 127, diskOffset);
	shooter->runIntake();
	drive->moveTo({2.5_tile, 0_tile}, 0_s, true);
	drive->moveTo({2.5_tile, 0.5_tile});
	drive->pointAt(goal, 0_s, false, true);
	shooter->multiShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	drive->moveTo({2.5_tile, 0_tile}, 0_s, true);
	shooter->runIntake();
	drive->moveTo({2_tile, -2_tile});
	drive->moveTo({5_tile, -2_tile}, 10_s, true, 30);
	roller->setState(atum8::Roller::RollerState::TurningToColor);
	drive->moveTo({2_tile, -2_tile});
	drive->moveTo({2_tile, -5_tile}, 10_s, true, 30);
	roller->setState(atum8::Roller::RollerState::TurningToColor);
	drive->moveTo({2_tile, -2_tile});
	drive->pointAt({0_in, 0_in});
	endGame->set_value(1);
}

void match()
{
	const atum8::Position start{-3_tile + 9_in, 0.5_tile, 90_deg};
	odometry->setPosition(atum8::accountForSide(start, autonSelector->getColor(), true));
	shooter->singleShotPrepare(2725_rpm);
	drive->moveTo({-2.5_tile, 0.5_tile}, 0.5_s);
	shooter->raiseIntake();
	drive->moveTo({-1.5_tile, 1.5_tile}, 6_s, false, 80, 127, diskOffset);
	getThreeStack();
	drive->moveTo({-2_tile, 1_tile}, 4_s, true);
	drive->pointAt(goal, 1.5_s);
	shooter->singleShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	shooter->singleShotPrepare(2730_rpm);
	shooter->raiseIntake();
	drive->moveTo({-1.5_tile, 0.5_tile}, 4_s, false, 80, 127, diskOffset);
	getThreeStack();
	drive->moveTo({-1.5_tile, 0.5_tile}, 1.5_s);
	drive->pointAt(goal);
	shooter->singleShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	shooter->runIntake();
	drive->moveTo({-2.4_tile, -0.4_tile}, 6_s, false, 55);
	drive->moveTo({-2_tile, 0_tile}, 1_s, true);
	drive->moveTo({-1.5_tile, 0.5_tile}, 4_s);
	drive->pointAt(goal, 3_s);
	shooter->singleShot(shooter->getDisks());
	atum8::waitFor(doneShooting);
	shooter->runIntake();
	drive->moveTo({0.5_tile, -1.5_tile}, 4_s, false, 80);
}

void special()
{
}

void getThreeStack()
{
	for (int i = 0; i < 3; i++)
	{
		shooter->raiseIntake();
		pros::delay(500);
		shooter->runIntake();
		pros::delay(500);
	}
}
