/**
 * @file intake.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for the intake subsystem.
 * @version 0.2
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
    class Intake : public Task {
    public:
    void taskFn();
    void controller();
    void diskControlls();
    void rollerControlls();
    private:
    static int buttonDuration;
    const short int longPress{250};
    bool isDiskMode{ true };
};
}