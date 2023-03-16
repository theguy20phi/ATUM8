#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"

namespace atum8 {

pros::Controller Chris(CONTROLLER_MASTER);

pros::Motor rightFrontDrive(17, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightMiddleDrive(13, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor rightBackDrive(16, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);

pros::Motor leftFrontDrive(14, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftMiddleDrive(12, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
pros::Motor leftBackDrive(18, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);

pros::Motor rightCatapultMotor(4, pros::E_MOTOR_GEAR_RED, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftCatapultMotor(2, pros::E_MOTOR_GEAR_RED, false, MOTOR_ENCODER_DEGREES);

pros::Motor leftIntakeMotor(10, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightIntakeMotor(8, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);

pros::ADIDigitalIn catapultStop('A');
pros::ADIDigitalOut endGame('B');

pros::Imu imuSensorAlpha(20);
pros::Imu imuSensorBeta(15);
pros::Imu imuSensorCharlie(11);
pros::Optical opticalSensor(9);
pros::Vision visionSensor(7);

pros::Rotation rightEncoder('D');
pros::Rotation leftEncoder('E');
pros::Rotation backEncoder('F');

int program{0};
} // namespace atum8