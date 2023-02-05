/**
 * @file constants.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides a set of common constants, data structures, and
 * utility functions.
 * @version 0.3
 * @date 2023-02-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <memory>
#include <array>
#include "pros/motors.hpp"
#include "pros/imu.hpp"

namespace atum8
{
    /* -------------------------------------------------------------------------- */
    /*                         Autonomous Routine Registry                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Enum for the possible autonomous routines.
     *
     */
    enum Routine
    {
        Left,
        Right,
        Center,
        Special,
        Skills
    };

    static constexpr std::array routineNames{"Left", "Right", "Center", "Special", "Skills"};

    static constexpr std::array routineDescs{
        "Left side autonomous routine.\nYou should put more info here!\nAnd here as well.",
        "Right side autonomous routine.\nLike where should it go?\nWhat does it do?",
        "Center autonomous routine.\nThis took a lot of time...\nMAKE USE OF IT!",
        "Special autonomous routine.\nSpecial times call for...\nspecial...\nmeasures...",
        "Skills autonomous.\nYou get the drill by now...\nPut more info here!"};

    /* -------------------------------------------------------------------------- */
    /*                             Unlikely to Change                             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Enum for different available colors.
     *
     */
    enum Color
    {
        Red,
        Blue
    };

    /**
     * @brief Struct to store match information.
     *
     */
    struct MatchInfo
    {
        Color color;
        Routine routine;
    };

    /**
     * @brief Max number of horizontal characters on brain screen.
     *
     */
    static constexpr int brainScreenWidth{32};

    /**
     * @brief The standard delay in tasks and the like.
     *
     */
    static constexpr int stdDelay{10};

    using UPMotor = std::unique_ptr<pros::Motor>;
    using UPImu = std::unique_ptr<pros::Imu>;
}