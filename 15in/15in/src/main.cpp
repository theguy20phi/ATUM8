#include "main.h"
#include "okapi/api/units/QLength.hpp"

atum8::SPGui gui;
atum8::SPGui autonSelector;
atum8::SPGui debugger;
atum8::SPMecanum drive;

void initialize()
{
	/* -------------------------------------------------------------------------- */
	/*                             GUI Initialization                             */
	/* -------------------------------------------------------------------------- */
	pros::lcd::initialize();
	pros::lcd::set_background_color(lv_color_t{0x000000});
	pros::lcd::set_text_color(255, 255, 255);
	autonSelector = std::make_shared<atum8::AutonSelector>();
	debugger = std::make_shared<atum8::Debugger>(atum8::Debugger::LineFns{
		[] (int control) { return "HELLO WORLD!"; },
		[] (int control) { return "IT'S YA BOY:"; },
		[] (int control) { return "<we don't have a name yet>"; }
	});
	gui = autonSelector;
	pros::lcd::register_btn0_cb([]()
								{ gui->control(-1); });
	pros::lcd::register_btn1_cb([]()
								{ gui->control(0); });
	pros::lcd::register_btn2_cb([]()
								{ gui->control(1); });

	drive = std::make_unique<atum8::Mecanum>(
		std::make_unique<pros::Motor>(17, true),
		std::make_unique<pros::Motor>(19),
		std::make_unique<pros::Motor>(12),
		std::make_unique<pros::Motor>(14, true),
		atum8::Mecanum::Dimensions{18_in, 12.56_in},
		nullptr, nullptr,
		std::make_unique<atum8::SettledChecker<okapi::QLength, okapi::QSpeed>>(2_in, 1_inps, 0.5_s),
		std::make_unique<atum8::SettledChecker<okapi::QAngle, okapi::QAngularSpeed>>(2_deg, 10_degps, 0.5_s),
		std::make_unique<atum8::SlewRate>(1),
		std::make_unique<atum8::SlewRate>(1),
		std::make_unique<pros::Imu>(1));
}

void disabled()
{
	gui = debugger;
	gui->view();
	while (true)
		pros::delay(10);
}

void autonomous() {}

enum class CatapultState
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
		drive->move(forward, strafe, turn);

		catapult.move_voltage((catapultStateMachine.getState() == CatapultState::Ready) ? 0 : 12000);
		catapultStateMachine.transition();

		pros::delay(10);
	}
}
