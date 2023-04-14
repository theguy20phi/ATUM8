/**
 * @file data.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for displaying debugging data to the brain screen.  
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"
#include "atum8/globals.hpp"
#include "atum8/misc/task.hpp"

namespace atum8{
    class Debugger : public Task{
        public:
        void taskFn();
        void displayCoordinates();
    };
}