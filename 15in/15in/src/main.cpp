#include "main.h"

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

enum class CatapultState
{
	Ready,
	Shooting,
	Preparing
};

void opcontrol()
{
	auto master = std::make_shared<pros::Controller>(pros::E_CONTROLLER_MASTER);

	pros::Motor lfWheel{19};
	pros::Motor lbWheel{12};
	pros::Motor rfWheel{17, true};
	pros::Motor rbWheel{14, true};

	pros::Motor catapultL{2, pros::motor_gearset_e_t::E_MOTOR_GEAR_RED};
	pros::Motor catapultR{4, pros::motor_gearset_e_t::E_MOTOR_GEAR_RED, true};
	pros::MotorGroup catapult{catapultL, catapultR};
	pros::ADIDigitalIn catapultSwitch{'A'};
	ATUM8::StateMachine<CatapultState>::Transitions catapultTransitions{
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
	ATUM8::StateMachine<CatapultState> catapultStateMachine{CatapultState::Preparing, catapultTransitions};
	catapult.set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	int counter{0};
	while (true)
	{
		const int strafe{100 * master->get_analog(ANALOG_LEFT_X)};
		const int forward{100 * master->get_analog(ANALOG_LEFT_Y)};
		const int turn{100 * master->get_analog(ANALOG_RIGHT_X)};
		lfWheel.move_voltage(forward + strafe + turn);
		lbWheel.move_voltage(forward - strafe + turn);
		rfWheel.move_voltage(forward - strafe - turn);
		rbWheel.move_voltage(forward + strafe - turn);

		catapult.move_voltage((catapultStateMachine.getState() == CatapultState::Ready) ? 0 : 12000);
		catapultStateMachine.transition();
		/*
		catapult.move_voltage((catapultState == CatapultState::Ready) ? 0 : 12000);
		switch(catapultState) {
			case CatapultState::Ready:
				if(master.get_digital_new_press(DIGITAL_R1)) {
					catapultState = CatapultState::Shooting;
				}
				break;
			case CatapultState::Shooting:
				if(!catapultSwitch.get_value())
					catapultState = CatapultState::Preparing;
				break;
			case CatapultState::Preparing:
				if(catapultSwitch.get_value())
					catapultState = CatapultState::Ready;
				break;
		};
		*/
		pros::delay(10);
	}
}
