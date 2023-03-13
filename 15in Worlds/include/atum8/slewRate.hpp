#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8{
    class SlewRate {
        public:
        //void slew(double targetValue, ) {

        //}
        double getOutput() {
            return output;
        }
        private:
        double output;
    };
}