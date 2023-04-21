/**
 * @file imus.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for the GPS Sensor. The methods are for updating the position of the robot. 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#pragma once
#include "main.h"
#include "atum8/globals.hpp"
#include "atum8/misc/Task.hpp"

namespace atum8{
    class GPS : public Task {
        public:
        void taskFn();
        private:
        pros::c::gps_status_s_t status;
    };
}