/**
 * @file odometry.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for a Bang Bang controll loop. 
 * @version 0.1
 * @date 2023-04-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"
#include "atum8/globals.hpp"

namespace atum8{
    class BangBang {
        public: 
        BangBang(bool hasThreshold_);
        BangBang(bool hasThreshold_, double threshold_);
        
        bool getOutput(double current, double desired);
        private:
        bool hasThreshold;
        double threshold;
    };
}