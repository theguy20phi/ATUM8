/**
 * @file programmingSkillsAuton.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the global function for the programming skills autonomous.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "atum8\algorithms\odometry.hpp"
#include "atum8\globals.hpp"
#include "atum8\systems\catapult.hpp"
#include "atum8\systems\drive.hpp"
#include "atum8\systems\endGame.hpp"
#include "atum8\systems\intake.hpp"
#include "atum8\sensors\vision.hpp"
#include "main.h"

namespace atum8 {
void programmingSkillsAuton();
} // namespace atum8