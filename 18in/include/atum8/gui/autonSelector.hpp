/**
 * @file autonSelector.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides the autonomous routine selector.
 * @version 0.3
 * @date 2023-02-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "gui.hpp"
#include <iostream>
#include <vector>

namespace atum8
{
    /**
     * @brief Provides all of the necessary methods and
     * state in order to select an autonomous routine at the beginning
     * of a match. Must set callbacks of LCD to appropriate control commands
     * (use anonymous functions capturing a pointer to an AutonSelector).
     * Change
     *
     */
    class AutonSelector : public Gui
    {
    public:
        /**
         * @brief Construct a new AutonSelector object
         *
         */
        AutonSelector();

        /**
         * @brief Handles the control of the AutonSelector and calls view to update
         * the screen.
         *
         * @param i The left button should have the callback [] () { autonSelector->control(-1); }
         * The middle button should have the callback [] () { autonSelector->control(0); }
         * The right button should have the callback [] () { autonSelector->control(1); }
         */
        void control(int i);

        /**
         * @brief Updates the LCD display to reflect the current match information.
         *
         */
        void view() const;

        /**
         * @brief Get the current match info.
         *
         * @return MatchInfo
         */
        MatchInfo getMatchInfo() const;

    private:
        void changeMatchInfo(int i);
        MatchInfo matchInfo{(Color)0, (Routine)0};
    };

    using UPAutonSelector = std::unique_ptr<AutonSelector>;
    using SPAutonSelector = std::shared_ptr<AutonSelector>;
}