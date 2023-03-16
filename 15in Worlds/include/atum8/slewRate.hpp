/**
 * @file slewRate.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for a simple slew rate. 
 * @version 0.1
 * @date 2023-03-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8{
    class SlewRate{
        public:
        double getOutput(float current, float desired, float accelerationStepUp);
        private:
        float output;
        float accelerationStepDown{1000};
    };
}