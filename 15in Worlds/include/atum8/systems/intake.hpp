/**
 * @file intake.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for the intake subsystem.
 * @version 0.4
 * @date 2023-04-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "atum8/globals.hpp"
#include "atum8/misc/task.hpp"
#include "main.h"

namespace atum8 {
    class Intake : public Task {
    public:
    void taskFn();
    void controller();
    void diskControlls();
    void rollerControlls();
    void setRollerToRed();
    void setRollerToBlue();
    private:
    const short int longPress{ 250 };
    bool isDiskMode{ true };
    bool is3StackMode { true };
    const float redRollerHue { 42.50 };
    const float blueRollerHue { 89.25 };
    const float rollerColorThreshold { 15 };
};
}