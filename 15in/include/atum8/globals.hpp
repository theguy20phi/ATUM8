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
extern pros::ADIDigitalIn catapultStop;
extern pros::Imu imuSensorAlpha;
extern pros::Imu imuSensorBeta;
extern pros::Imu imuSensorCharlie;
extern pros::Optical opticalSensor;
extern pros::ADIDigitalOut endGame;

//
const double encoderWheelRadius{1.625}; // inches
const double driveGearRatio {60.00000000000000000 / 36.00000000000000000};
extern int program;
} // namespace atum8