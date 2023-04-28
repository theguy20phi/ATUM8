/**
 * @file imus.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides a class for the IMU Sensors. The methods simpley combines the IMU Sensors together.
 * @version 0.2
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#pragma once
#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "atum8/globals.hpp"

namespace atum8{
    class Imus {
        public:
        void resetImuSensors();
        double getImuSensorAverages();
        void calibrateImuSensors();
        double getDeltaHeadingImu();
        private:
        double prevHeading { 0 };
    };
}