/**
 * @file constants.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides a set of common constants, data structures, and
 * utility functions.
 * @version 0.4
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
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "okapi/api/units/QTime.hpp"

using namespace okapi::literals;

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
        Match,
        Skills,
        Special
    };

    static constexpr std::array routineNames{"Match", "Skills", "Special"};

    static constexpr std::array routineDescs{
        "Normal match routine.",
        "Programming skills routine.",
        "Special autonomous routine.\nLikely used for testing."};

    /* -------------------------------------------------------------------------- */
    /*                             Unlikely to Change                             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Will wait for a specific condition to be true, or until max time is used (if
     * it is set).
     *
     * @param condition
     * @param maxTime
     */
    void waitFor(const std::function<bool()> &condition, const okapi::QTime &maxTime = 0_s);

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
     * @brief Max number of horizontal characters on brain screen LCD.
     *
     */
    static constexpr int brainScreenWidth{32};

    /**
     * @brief Max number of rows on brain screen LCD.
     *
     */
    static constexpr int brainScreenHeight{8};

    /**
     * @brief The standard delay in tasks and the like.
     *
     */
    static constexpr int stdDelay{10};

    using UPMotor = std::unique_ptr<pros::Motor>;
    using UPMotorGroup = std::unique_ptr<pros::MotorGroup>;
    using UPADIDigitalOut = std::unique_ptr<pros::ADIDigitalOut>;
    using SPADIDigitalOut = std::shared_ptr<pros::ADIDigitalOut>;
    using UPADIAnalogIn = std::unique_ptr<pros::ADIAnalogIn>;
    using UPOptical = std::unique_ptr<pros::Optical>;
    using UPVision = std::unique_ptr<pros::Vision>;
}