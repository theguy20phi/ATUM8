/**
 * @file globals.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the setup for the motors, controller, and sensors. 
 *        It also provides globals variables used throughout the program. 
 * @version 0.5
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

namespace atum8 {
// Robot Setup
extern pros::Controller Chris;
extern pros::Motor rightFrontTopDrive;
extern pros::Motor rightFrontBotDrive;
extern pros::Motor rightBackDrive;
extern pros::Motor leftFrontTopDrive;
extern pros::Motor leftFrontBotDrive;
extern pros::Motor leftBackDrive;

extern pros::Motor rightCatapultMotor;
extern pros::Motor leftCatapultMotor;

extern pros::Motor leftIntakeMotor;
extern pros::Motor rightIntakeMotor;

extern pros::Motor_Group leftDriveMotors;
extern pros::Motor_Group rightDriveMotors;
extern pros::Motor_Group driveMotors;

extern pros::Motor_Group catapultMotors;
extern pros::Motor_Group intakeMotors;

extern pros::GPS gpsSensor;
extern pros::Imu imuSensorAlpha;
extern pros::Imu imuSensorBeta;
extern pros::Imu imuSensorCharlie;
extern pros::Optical opticalSensor;
extern pros::Vision visionSensorGoal;
extern pros::Vision visionSensorDisk;
extern pros::ADIDigitalIn catapultStop;
extern pros::ADIDigitalOut endGameRight;
extern pros::ADIDigitalOut endGameLeft;
extern pros::ADIEncoder rightEncoder;
extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder backEncoder;
extern pros::ADIAnalogOut intakeToggler;

//
const float encoderWheelRadius{1.625}; // inches
const double encoderWheelCircumference{203.09/25.4};//{200/25.4};


const double driveGearRatio {60.0 / 36.0};
extern int program;
extern double globalX;
extern double globalY;
extern double globalHeadingInRadians;
extern double globalHeadingInDegrees;
extern double globalLinearPower;
extern double globalTurnPower;
extern bool globalIsCatapultManualMode;
extern bool isRedAimBotMode;
extern pros::Mutex positionMutex;
} // namespace atum8