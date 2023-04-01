#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"

namespace atum8 {

pros::Controller Chris(CONTROLLER_MASTER);

// Setup Motors
pros::Motor rightFrontTopDrive(13, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor rightFrontBotDrive(12, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightBackDrive(11, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);

pros::Motor leftFrontTopDrive(17, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
pros::Motor leftFrontBotDrive(16, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftBackDrive(15, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);

pros::Motor rightCatapultMotor(18, pros::E_MOTOR_GEAR_RED, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftCatapultMotor(19, pros::E_MOTOR_GEAR_RED, false, MOTOR_ENCODER_DEGREES);

pros::Motor rightIntakeMotor(1, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftIntakeMotor(2, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);

// Setup Motor Groups
pros::Motor_Group leftDriveMotors ({leftFrontTopDrive, leftFrontBotDrive, leftBackDrive});
pros::Motor_Group rightDriveMotors({rightFrontTopDrive, rightFrontBotDrive, rightBackDrive});
pros::Motor_Group driveMotors({rightFrontTopDrive, rightFrontBotDrive, rightBackDrive, leftFrontTopDrive, leftFrontBotDrive, leftBackDrive});

pros::Motor_Group catapultMotors({rightCatapultMotor, leftCatapultMotor});

pros::Motor_Group intakeMotors({rightIntakeMotor, leftIntakeMotor});

// Setup Sensors
pros::Imu imuSensorAlpha(20);
pros::Imu imuSensorBeta(15);
pros::Imu imuSensorCharlie(11);
pros::Optical opticalSensor(9);
pros::Rotation rotationSensor(20);
pros::Vision visionSensor(6);

pros::ADIDigitalIn guiToggle('A');
pros::ADIDigitalOut endGame('B');
pros::ADIEncoder leftEncoder('D', 'C', false);
pros::ADIEncoder rightEncoder('E', 'F', false);
pros::ADIEncoder backEncoder('G', 'H', false);

int program{0};
double globalX;
double globalY;
double globalHeadingInRadians;
double globalHeadingInDegrees;
} // namespace atum8