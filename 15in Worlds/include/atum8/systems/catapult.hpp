/**
 * @file catapult.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for the catapult subsystem.
 * @version 0.3
 * @date 2023-04-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "atum8/globals.hpp"
#include "atum8/misc/task.hpp"
#include "main.h"

namespace atum8 {
    class Catapult : public Task {
    public:
    void taskFn();
    void controller();
    void automaticContolls();
    void manualControlls();
    bool getCatapultMode();
    private:
    bool isManualMode { false };
};
}