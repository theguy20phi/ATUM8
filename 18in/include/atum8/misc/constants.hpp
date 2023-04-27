#pragma once

#include <memory>
#include <array>
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "pros/gps.hpp"
#include "okapi/api/units/QTime.hpp"

using namespace okapi::literals;

namespace atum8
{
    /* -------------------------------------------------------------------------- */
    /*                         Autonomous Routine Registry                        */
    /* -------------------------------------------------------------------------- */
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

    void waitFor(const std::function<bool()> &condition, const okapi::QTime &maxTime = 0_s);

    enum Color
    {
        Red,
        Blue
    };

    struct MatchInfo
    {
        Color color;
        Routine routine;
    };

    static constexpr int brainScreenWidth{32};

    static constexpr int brainScreenHeight{8};

    static constexpr int stdDelay{10};

    using UPMotor = std::unique_ptr<pros::Motor>;
    using UPMotorGroup = std::unique_ptr<pros::MotorGroup>;
    using UPADIDigitalOut = std::unique_ptr<pros::ADIDigitalOut>;
    using SPADIDigitalOut = std::shared_ptr<pros::ADIDigitalOut>;
    using UPADIAnalogIn = std::unique_ptr<pros::ADIAnalogIn>;
    using UPOptical = std::unique_ptr<pros::Optical>;
    using UPVision = std::unique_ptr<pros::Vision>;
    using UPRawGPS = std::unique_ptr<pros::GPS>;
    using UPADILED = std::unique_ptr<pros::ADILED>;
}