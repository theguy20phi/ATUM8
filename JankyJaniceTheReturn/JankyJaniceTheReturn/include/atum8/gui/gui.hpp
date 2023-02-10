#pragma once

#include "pros/llemu.hpp"
#include "atum8/constants.hpp"
#include <string>
#include <memory>

namespace atum8
{
    class Gui
    {
    public:
        /**
         * @brief Handles the control of the screen.
         *
         * @param i The left button should have the callback [] () { gui->control(-1); }
         * The middle button should have the callback [] () { gui->control(0); }
         * The right button should have the callback [] () { gui->control(1); }
         */
        virtual void control(int i) = 0;

        /**
         * @brief Updates the LCD display to reflect the current match information.
         *
         */
        virtual void view() const = 0;

    protected:
        void incrementCursor(int loop);
        void printDesc(int line, const std::string &concatLines) const;
        int cursor{0};
    };

    using UPGui = std::unique_ptr<Gui>;
    using SPGui = std::shared_ptr<Gui>;
}