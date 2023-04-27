/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"
#include "okapi/api.hpp"
#include "atum8/atum8.hpp"

atum8::SPGui gui;
atum8::SPAutonSelector autonSelector;
atum8::SPDebugger debugger;
atum8::SPDrive drive;
atum8::SPShooter shooter;
atum8::SPRoller roller;
atum8::UPADIDigitalOut endGame;
atum8::SPOdometry odometry;

void initializeLCD();
void skills();
void match();
void special();
void turnRoller();
void getThreeStack();

static constexpr atum8::Position goal{2_tile + 4.5_in, 2_tile + 4.5_in};
static constexpr okapi::QLength diskOffset{8.75_in};

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C"
{
#endif
    void autonomous(void);
    void initialize(void);
    void disabled(void);
    void competition_initialize(void);
    void opcontrol(void);
#ifdef __cplusplus
}
#endif
