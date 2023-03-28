/**
 * @file odometry.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for absolute position tracking. (Odometry very cool)
 * @version 0.3
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"
#include "globals.hpp"

namespace atum8{
    class Odometry {
        public:

        void trackPosition();
        void setStartingPosition(double x, double y);
        double getPosition();

        private:
        double multiplier {1.0 / 2048 / 4};
        const float sR{ 7.25 };
        const float sL { 7.25 };
        const float sS { 2.5 };

        double deltaRightPosition;
        double prevRightPosition;

        double deltaLeftPosition;
        double prevLeftPosition;

        double deltaBackPosition;
        double prevBackPosition;

        double deltaHeading;
        double currentHeading;
        double prevHeading;

        double headingAverage;
        double deltaX;
        double deltaY;
    };
}