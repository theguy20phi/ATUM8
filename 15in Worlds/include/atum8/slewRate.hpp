#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8{
    class SlewRate{
        public:
        double getOutput(float current, float desired, float accelerationStepUp);
        private:
        float output;
        float accelerationStepDown{3000};
    };
}