/**
 * @file atum8.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Includes all the other files in the atum8 directory.
 * @version 0.2
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "systems/drive.hpp"
#include "systems/shooter.hpp"
#include "systems/roller.hpp"
#include "misc/constants.hpp"
#include "misc/stateMachine.hpp"
#include "misc/slewRate.hpp"
#include "misc/settledChecker.hpp"
#include "misc/task.hpp"
#include "filters/filter.hpp"
#include "filters/rollingAverage.hpp"
#include "filters/rollingMedian.hpp"
#include "gui/gui.hpp"
#include "gui/autonSelector.hpp"
#include "gui/debugger.hpp"
#include "controllers/controller.hpp"
#include "controllers/bangBang.hpp"
#include "controllers/tbh.hpp"
#include "controllers/pidFF.hpp"
#include "controllers/slider.hpp"
#include "devices/poseEstimator.hpp"
#include "devices/odometry.hpp"
#include "devices/odometer.hpp"
#include "devices/position.hpp"
#include "devices/imus.hpp"
#include "devices/potentiometer.hpp"