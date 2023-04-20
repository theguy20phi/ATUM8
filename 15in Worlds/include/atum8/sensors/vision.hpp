/**
 * @file Vision.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for aim bot capaibilities. 
 * @version 0.4
 * @date 2023-03-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"
#include "atum8/globals.hpp"
#include "atum8/systems/drive.hpp"
#include "atum8/algorithms/pid.hpp"

namespace atum8{
    class Vision : Drive{
        public:
        void redAimBot();
        void blueAimBot();
        void diskAimBot();


        private:
    };
}