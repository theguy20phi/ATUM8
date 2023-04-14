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
#include "atum8/globals.hpp"
#include "atum8/misc/task.hpp"

namespace atum8{
    class Odometry : public Task{
        public:

        void taskFn();
        void trackPosition();
        void setStartingPosition(double x, double y, double headingInDegrees);
        double getWorldX();
        double getWorldY();
        double getWorldHeadingInRadians();
        double getWorldHeadingInDegrees();

        private:
        double multiplier {1.0 / 2048 / 4};
        const float sR{ 6.69291 };
        const float sL { 6.69291 };
        const float sS { 0.905512 };

        double rightPosition;
        double leftPosition;
        double backPosition;

        double deltaRightPosition;
        double prevRightPosition { 0 };

        double deltaLeftPosition;
        double prevLeftPosition { 0 };

        double deltaBackPosition;
        double prevBackPosition { 0 };

        double deltaHeading;
        double currentHeading;
        double prevHeading;

        double headingAverage;
        double deltaX;
        double deltaY;

        double worldX;
        double worldY;
        double worldHeadingInRadians;
        double worldHeadingInDegrees;
    };
}