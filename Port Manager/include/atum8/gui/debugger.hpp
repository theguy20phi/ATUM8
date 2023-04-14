#pragma once

#include "gui.hpp"
#include <functional>

namespace atum8
{
    class Debugger : public Gui
    {
    public:
        using LineFn = std::function<std::string(int)>;
        using LineFns = std::vector<LineFn>;

        /**
         * @brief Constructs a new Debugger object.
         *
         * @param iLineFns A vector of functions taking an integer and returning a string.
         * The string will be displayed and other operations can take place.
         */
        Debugger(const LineFns &iLineFns);

        /**
         * @brief Handles the control of the Debugger and calls view to update
         * the screen.
         *
         * @param i The left button should have the callback [] () { Debugger->control(-1); }
         * The middle button should have the callback [] () { Debugger->control(0); }
         * The right button should have the callback [] () { Debugger->control(1); }
         */
        void control(int i);

        /**
         * @brief Updates the LCD display to reflect the current LineFns in view.
         *
         */
        void view() const;

    private:
        const LineFns lineFns;
    };

    using UPDebugger = std::unique_ptr<Debugger>;
    using SPDebugger = std::shared_ptr<Debugger>;
}