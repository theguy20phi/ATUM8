#pragma once

#include "gui.hpp"
#include <iostream>
#include <vector>

namespace atum8
{
    class AutonSelector : public Gui
    {
    public:
        AutonSelector();

        void control(int i);

        void view() const;

        MatchInfo getMatchInfo() const;

        void setColor(const Color &color);

        Color getColor() const;

    private:
        void changeMatchInfo(int i);
        MatchInfo matchInfo{(Color)0, (Routine)0};
    };

    using UPAutonSelector = std::unique_ptr<AutonSelector>;
    using SPAutonSelector = std::shared_ptr<AutonSelector>;
}