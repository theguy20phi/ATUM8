#pragma once
#include "globals.hpp"
#include "main.h"

namespace atum8{
    class Imus {
        public:
        void resetImuSensors();
        double getImuSensorAverages();
    };
}