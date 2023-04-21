#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"

namespace atum8 {

pros::Controller Chris(CONTROLLER_MASTER);

// Setup Motors
pros::Motor rightFrontTopDrive(13, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//13
pros::Motor rightFrontBotDrive(12, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//12
pros::Motor rightBackDrive(11, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//11

pros::Motor leftFrontTopDrive(17, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//17
pros::Motor leftFrontBotDrive(16, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//16
pros::Motor leftBackDrive(15, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);//15

pros::Motor rightCatapultMotor(18, pros::E_MOTOR_GEAR_RED, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftCatapultMotor(19, pros::E_MOTOR_GEAR_RED, false, MOTOR_ENCODER_DEGREES);

pros::Motor rightIntakeMotor(1, pros::E_MOTOR_GEAR_GREEN, true, MOTOR_ENCODER_DEGREES);//1
pros::Motor leftIntakeMotor(2, pros::E_MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);

// Setup Motor Groups
pros::Motor_Group leftDriveMotors ({leftFrontTopDrive, leftFrontBotDrive, leftBackDrive});
pros::Motor_Group rightDriveMotors({rightFrontTopDrive, rightFrontBotDrive, rightBackDrive});
pros::Motor_Group driveMotors({rightFrontTopDrive, rightFrontBotDrive, rightBackDrive, leftFrontTopDrive, leftFrontBotDrive, leftBackDrive});

pros::Motor_Group catapultMotors({rightCatapultMotor, leftCatapultMotor});

pros::Motor_Group intakeMotors({rightIntakeMotor, leftIntakeMotor});

// Setup Sensors
pros::GPS gpsSensor(19);
pros::Imu imuSensorAlpha(20);
pros::Imu imuSensorBeta(15);
pros::Imu imuSensorCharlie(11);
pros::Optical opticalSensor(9);
pros::Vision visionSensorGoal(20);
pros::Vision visionSensorDisk(10);

pros::ADIDigitalIn catapultStop('G');
pros::ADIEncoder rightEncoder('A', 'B', false);
pros::ADIEncoder leftEncoder('C', 'D', false);
pros::ADIEncoder backEncoder('E', 'F', false);
pros::ADIDigitalOut endGameRight({{14, 'A'}});
pros::ADIDigitalOut endGameLeft ({{14, 'B'}});

int program{ 0 };
double globalX;
double globalY;
double globalHeadingInRadians;
double globalHeadingInDegrees;
double globalLinearPower;
double globalTurnPower;
bool globalIsCatapultManualMode { false };
bool isRedAimBotMode { true };
pros::Mutex positionMutex;
} // namespace atum8