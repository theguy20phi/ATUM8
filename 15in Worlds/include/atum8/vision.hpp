/**
 * @file Vision.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for aim bot capaibilities. 
 * @version 0.2
 * @date 2023-03-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"
#include "globals.hpp"

namespace atum8{
    class Vision {
        public:
        void redAimBot();
        void blueAimBot();


        private:
        const short int redID { 1 };
        const short int blueID { 2 };
        const short int visionWidth { 316 };
        const short int visionHeight { 212 };
    };
}