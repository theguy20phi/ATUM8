#include "main.h"
#include "okapi/api/units/QLength.hpp"

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
		{ return "JANKY JANICE!!!"; }});
	gui = autonSelector;
	gui->view();

	drive = atum8::SPMecanumBuilder()
				.withRFMotor({20, -19})
				.withLFMotor({-10, 8})
				.withLBMotor({2, -1})
				.withRBMotor({11, -12})
				.withBaseWidth(18_in)
				.withWheelCircum(12.56_in)
				.withForwardSettledChecker(2_in, 1_inps, 0.5_s)
				.withTurnSettledChecker(2_deg, 10_degps, 0.5_s)
				.withStickFunction([](int input)
								   { return pow(input, 3) / 16129; })
				.withBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD)
				.build();

	roller = atum8::SPRollerBuilder()
				 .withMotor(4)
				 .build();
}

void disabled()
{
	gui = autonSelector;
	gui->view();
	while (true)
		pros::delay(10);
}

void autonomous() {}

void opcontrol()
{
	gui = debugger;
	gui->view();
	pros::Controller master{pros::controller_id_e_t::E_CONTROLLER_MASTER};
	bool manual{false};
	while (true)
	{
		const int forward{master.get_analog(ANALOG_LEFT_Y)};
		const int strafe{0};
		const int turn{master.get_analog(ANALOG_RIGHT_X)};
		drive->driver(forward, strafe, turn);

		if (master.get_digital_new_press(DIGITAL_B))
			manual = !manual;
		if (manual)
		{
			if (master.get_digital(DIGITAL_R1))
				roller->runRoller(127);
			else
				roller->runRoller(0);
		} else if(master.get_digital_new_press)

		pros::delay(atum8::stdDelay);
	}
}

/*enum class CatapultState
{
	Ready,
	Shooting,
	Preparing
};

void opcontrol()
{
	gui = debugger;
	gui->view();

	auto master{std::make_shared<pros::Controller>(CONTROLLER_MASTER)};

	pros::Motor catapultL{2, pros::motor_gearset_e_t::E_MOTOR_GEAR_RED};
	pros::Motor catapultR{4, pros::motor_gearset_e_t::E_MOTOR_GEAR_RED, true};
	pros::MotorGroup catapult{catapultL, catapultR};
	pros::ADIDigitalIn catapultSwitch{'A'};
	atum8::StateMachine<CatapultState>::Transitions catapultTransitions{
		{CatapultState::Ready, [master]()
		 {
			 return master->get_digital_new_press(DIGITAL_R1) ? CatapultState::Shooting : CatapultState::Ready;
		 }},
		{CatapultState::Shooting, [catapultSwitch]()
		 {
			 return !catapultSwitch.get_value() ? CatapultState::Preparing : CatapultState::Shooting;
		 }},
		{CatapultState::Preparing, [catapultSwitch]()
		 {
			 return catapultSwitch.get_value() ? CatapultState::Ready : CatapultState::Preparing;
		 }}};
	atum8::StateMachine<CatapultState> catapultStateMachine{CatapultState::Preparing, catapultTransitions};
	catapult.set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	atum8::UPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> testSettledChecker{
		std::make_unique<atum8::SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(20_deg, 5_rpm, 1_s)};

	while (true)
	{
		const int forward{master->get_analog(ANALOG_LEFT_Y)};
		const int strafe{master->get_analog(ANALOG_LEFT_X)};
		const int turn{master->get_analog(ANALOG_RIGHT_X)};
		drive->driver(forward, strafe, turn);

		catapult.move_voltage((catapultStateMachine.getState() == CatapultState::Ready) ? 0 : 12000);
		catapultStateMachine.transition();

		pros::delay(10);
	}
}*/

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