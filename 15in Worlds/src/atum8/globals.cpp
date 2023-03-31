#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"

namespace atum8 {

pros::Controller Chris(CONTROLLER_MASTER);

// Setup Motors
pros::Motor rightFrontDrive(10, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//17
pros::Motor rightMiddleDrive(13, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//13
pros::Motor rightBackDrive(20, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//16

pros::Motor leftFrontDrive(1, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//14
pros::Motor leftMiddleDrive(12, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//12
pros::Motor leftBackDrive(11, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//18

pros::Motor rightCatapultMotor(4, pros::E_MOTOR_GEAR_RED, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftCatapultMotor(2, pros::E_MOTOR_GEAR_RED, false, MOTOR_ENCODER_DEGREES);

pros::Motor rightIntakeMotor(8, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftIntakeMotor(14, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//10

// Setup Motor Groups
pros::Motor_Group leftDriveMotors ({leftFrontDrive, leftMiddleDrive, leftBackDrive});
pros::Motor_Group rightDriveMotors({rightFrontDrive, rightMiddleDrive, rightBackDrive});
pros::Motor_Group driveMotors({rightFrontDrive, rightMiddleDrive, rightBackDrive, leftFrontDrive, leftMiddleDrive, leftBackDrive});

pros::Motor_Group catapultMotors({rightCatapultMotor, leftCatapultMotor});

pros::Motor_Group intakeMotors({rightIntakeMotor, leftIntakeMotor});

// Setup Sensors
//pros::ADIDigitalIn catapultStop('A');
//pros::ADIDigitalOut endGame('B');
pros::Imu imuSensorAlpha(20);
pros::Imu imuSensorBeta(15);
pros::Imu imuSensorCharlie(11);
pros::Optical opticalSensor(9);
pros::Vision visionSensor(6);

pros::ADIEncoder leftEncoder('D', 'C', false);
pros::ADIEncoder rightEncoder('E', 'F', false);
pros::ADIEncoder backEncoder('G', 'H', false);

int program{0};
double globalX;
double globalY;
double globalHeadingInRadians;
double globalHeadingInDegrees;
double globalRightPower;
double globalLeftPower;

} // namespace atum8