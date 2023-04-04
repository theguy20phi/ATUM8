/**
 * @file mathNStuff.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the helper functions that are used elsewhere.
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "atum8/globals.hpp"
#include "main.h"

namespace atum8 {
namespace utility {
double rpmToPower(double rpm);
double reduce_0_to_360(double angle);
double reduce_negative_180_to_180(double angle);
double reduce_negative_90_to_90(double angle);
double convertRadianToDegree(double angleRadian);
double convertDegreeToRadian(double angleDegree);
double clamp(double input, double min, double max);
} // namespace utility
} // namespace atum8