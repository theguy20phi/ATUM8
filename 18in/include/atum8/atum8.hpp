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

#include "systems/mecanum.hpp"
#include "systems/flywheel.hpp"
#include "systems/intake.hpp"
#include "systems/roller.hpp"
#include "constants.hpp"
#include "stateMachine.hpp"
#include "slewRate.hpp"
#include "settledChecker.hpp"
#include "imus.hpp"
#include "task.hpp"
#include "rollingAverage.hpp"
#include "gui/gui.hpp"
#include "gui/autonSelector.hpp"
#include "gui/debugger.hpp"
#include "controllers/controller.hpp"
#include "controllers/bangBang.hpp"
#include "controllers/tbh.hpp"
#include "controllers/pidFF.hpp"
#include "controllers/slider.hpp"