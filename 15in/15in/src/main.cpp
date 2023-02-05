#include "main.h"

atum8::UPAutonSelector autonSelector;
std::unique_ptr<atum8::Mecanum> drive;

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_background_color(lv_color_t{0x000000});
	pros::lcd::set_text_color(255, 255, 255);
	autonSelector = std::make_unique<atum8::AutonSelector>();
	drive = std::make_unique<atum8::Mecanum>(
		std::make_unique<pros::Motor>(17, true),
		std::make_unique<pros::Motor>(19),
		std::make_unique<pros::Motor>(12),
		std::make_unique<pros::Motor>(14, true),
		nullptr, nullptr,
		std::make_unique<pros::Imu>(1));
}

void disabled()
{
	pros::lcd::register_btn0_cb([]()
								{ autonSelector->control(-1); });
	pros::lcd::register_btn1_cb([]()
								{ autonSelector->control(0); });
	pros::lcd::register_btn2_cb([]()
								{ autonSelector->control(1); });
	autonSelector->view();
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
	pros::lcd::register_btn0_cb([]()
								{ autonSelector->control(-1); });
	pros::lcd::register_btn1_cb([]()
								{ autonSelector->control(0); });
	pros::lcd::register_btn2_cb([]()
								{ autonSelector->control(1); });
	autonSelector->view();

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
	atum8::StateMachine<CatapultState> catapultStateMachine{CatapultState::Preparing};
	catapult.set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	int counter{0};
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
