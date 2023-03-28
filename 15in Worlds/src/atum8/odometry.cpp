#include "atum8/odometry.hpp"
#include "globals.hpp"
#include "main.h"

namespace atum8 {
    void Odometry::trackPosition() {
        // Calculuate Delta Right, Delta Left, and Delta Back
        deltaRightPosition = ( (rightEncoder.get_position() * encoderWheelCircumference * multiplier) - prevRightPosition);
        deltaLeftPosition = ( (leftEncoder.get_position() * encoderWheelCircumference * multiplier) - prevLeftPosition);
        deltaBackPosition = ( (backEncoder.get_position() * encoderWheelCircumference * multiplier) - prevBackPosition);

        // Updating Previous variables
        prevRightPosition = rightEncoder.get_position();
        prevLeftPosition = leftEncoder.get_position();
        prevBackPosition = backEncoder.get_position();

        // Calculate Delta Heading and update previous heading variable
        currentHeading = (deltaLeftPosition - deltaRightPosition) / (sL + sR);
        deltaHeading = currentHeading - prevHeading;
        prevHeading = currentHeading; 

        // Calculate Delta X and Delta Y
        if(deltaHeading == 0) {
            deltaX = deltaBackPosition;
            deltaY = deltaRightPosition;
        } else {
            deltaX = 2 * sin(currentHeading / 2) * ((deltaBackPosition/deltaHeading) + sS);
            deltaY = 2 * sin(currentHeading / 2) * ((deltaRightPosition/deltaHeading) + sR);
        }

        // Calculate Heading Average
        headingAverage = globalHeadingInRadians + deltaHeading / 2;

        // Update Global X Coordinate, Y Coordinate, Heading in Radians, and Heading in Degrees
        globalX += deltaX * cos(headingAverage) + deltaY * sin(headingAverage);
        globalY += deltaY * cos(headingAverage) - deltaX * sin(headingAverage);
        globalHeadingInRadians += deltaHeading;
        globalHeadingInDegrees = globalHeadingInRadians * 180 / M_PI;
    }

    void Odometry::setStartingPosition(double x, double y) {
        globalX = x;
        globalY = y;
    }

    double Odometry::getPosition() {
        return true;//yup idk how to do this right now deal with it later me lol kek
    }
}