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
#include "atum8/controllers/pid.hpp"

namespace atum8{
    class Vision : Drive{
        public:
        void redAimBot();
        void blueAimBot();
        void diskAimBot();


        private:
        const short int redID { 1 };
        const short int blueID { 2 };
        const short int yellowID { 3 };
        const short int visionFOVWidth { 316 };
        const short int visionFOVHeight { 212 };
    };
}