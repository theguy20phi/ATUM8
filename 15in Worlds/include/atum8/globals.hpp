/**
 * @file globals.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the setup for the motors, controller, and sensors. 
 *        It also provides globals variables used throughout the program. 
 * @version 0.3
 * @date 2023-03-29
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
extern pros::Motor rightFrontDrive;
extern pros::Motor rightMiddleDrive;
extern pros::Motor rightBackDrive;
extern pros::Motor leftFrontDrive;
extern pros::Motor leftMiddleDrive;
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
extern pros::ADIDigitalIn catapultStop;
extern pros::Imu imuSensorAlpha;
extern pros::Imu imuSensorBeta;
extern pros::Imu imuSensorCharlie;
extern pros::Optical opticalSensor;
extern pros::Vision visionSensor;
extern pros::ADIDigitalOut endGame;
extern pros::ADIEncoder rightEncoder;
extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder backEncoder;

//
const float encoderWheelRadius{1.625}; // inches
const double encoderWheelCircumference{2.3 * M_PI};//{200/25.4};

const double driveGearRatio {60.0 / 36.0};
extern int program;
extern double globalX;
extern double globalY;
extern double globalHeadingInRadians;
extern double globalHeadingInDegrees;
extern double globalRightPower;
extern double globalLeftPower;
} // namespace atum8