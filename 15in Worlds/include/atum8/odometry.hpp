#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8{
    class Odometry {
        public:
        private:
        double getDeltaRight();
        double getDeltaLeft();
        double getDeltaBack();
        
        double getTheta();
        double getLocalX();
        double getLocalY();
        
        const float sR = 7.25;
        const float sL = 7.25;
        const float sS = 2.5;

    };
}