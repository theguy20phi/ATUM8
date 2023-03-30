/**
 * @file intake.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com)
 * @brief This file provides the class for the intake subsystem.
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "atum8/globals.hpp"
#include "atum8/task.hpp"
#include "main.h"

namespace atum8 {
    class Intake : public Task {
    public:
    void taskFn();
    void controller();
    private:
};
}