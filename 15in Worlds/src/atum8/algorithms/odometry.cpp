#include "atum8/algorithms/odometry.hpp"
#include "atum8/globals.hpp"
#include "main.h"

namespace atum8 {
    void Odometry::taskFn() {
        while(true) {
            trackPosition();
            pros::delay(10);
        }
    }

    void Odometry::trackPosition() {
        // Calculuate Delta Right, Delta Left, and Delta Back
        //std::cout << "Right: " << rightEncoder.get_value() << std::endl;
        //std::cout << "Left: " << leftEncoder.get_value() << std::endl;
        //std::cout << "Back: " << backEncoder.get_value() << std::endl;

        rightPosition = (rightEncoder.get_value() * encoderWheelCircumference * multiplier);
        leftPosition = (leftEncoder.get_value() * encoderWheelCircumference * multiplier);
        backPosition = (backEncoder.get_value() * encoderWheelCircumference * multiplier);

        deltaRightPosition = (rightPosition - prevRightPosition);
        deltaLeftPosition = (leftPosition - prevLeftPosition);
        deltaBackPosition = (backPosition - prevBackPosition);

        // Updating Previous variables
        prevRightPosition = rightPosition;
        prevLeftPosition = leftPosition;
        prevBackPosition = backPosition;

        // Calculate Delta Heading and update previous heading variable
        //const double deltaHeading = imuTrust * getDeltaHeadingImu() + (1.0 - imuTrust) * (deltaLeftPosition - deltaRightPosition) / (sL + sR);
        double deltaHeading = (deltaLeftPosition - deltaRightPosition) / (sL + sR);
        // Calculate Delta X and Delta Y
        if(deltaHeading == 0) {
            deltaX = deltaBackPosition;
            deltaY = deltaRightPosition;
        } else {
            deltaX = 2 * sin(deltaHeading / 2) * ((deltaBackPosition/deltaHeading) + sS);
            deltaY = 2 * sin(deltaHeading / 2) * ((deltaRightPosition/deltaHeading) + sR);
        }

        // Calculate Heading Average
        headingAverage = globalHeadingInRadians + deltaHeading / 2;

        // Update Global X Coordinate, Y Coordinate, Heading in Radians, and Heading in Degrees
        //positionMutex.take();
        globalX += deltaX * cos(headingAverage) + deltaY * sin(headingAverage);
        globalY += deltaY * cos(headingAverage) - deltaX * sin(headingAverage);

        globalHeadingInRadians += deltaHeading;
        globalHeadingInDegrees = globalHeadingInRadians * 180.0 / M_PI;
        //positionMutex.give();
    }

    void Odometry::setStartingPosition(double x, double y, double headingInDegrees) {
        globalX = x;
        globalY = y;
        globalHeadingInRadians = utility::convertDegreeToRadian(headingInDegrees);
        globalHeadingInDegrees = headingInDegrees;
    }

    void Odometry::setImuTrust(double trustFactor) {
        imuTrust = trustFactor;
    }
}