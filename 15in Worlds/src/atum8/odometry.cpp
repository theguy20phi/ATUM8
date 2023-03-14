#include "atum8/odometry.hpp"
#include "globals.hpp"
#include "main.h"

namespace atum8 {
    double Odometry::getDeltaRight(){
        return (encoderWheelCircumference * rightEncoder.get_position()) / 360;
    }

    double Odometry::getDeltaLeft(){
        return (encoderWheelCircumference * leftEncoder.get_position()) / 360;
    }

    double Odometry::getDeltaBack(){
        return (encoderWheelCircumference * backEncoder.get_position()) / 360;
    }

    double Odometry::getTheta(){
        return (getDeltaLeft() - getDeltaRight())/(sL + sR);
    }

     double Odometry::getLocalX(){
        return 2 * sin(getTheta()/2) * ((getDeltaBack() / getTheta()) + sS);
     }

     double Odometry::getLocalY() {
        return 2 * sin(getTheta()/2) * ((getDeltaRight() / getTheta()) / sR);
     }

    
}