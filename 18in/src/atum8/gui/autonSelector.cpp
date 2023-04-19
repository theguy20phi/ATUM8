#include "autonSelector.hpp"

namespace atum8
{
    AutonSelector::AutonSelector()
    {
        if (routineNames.size() != routineDescs.size())
            std::cout << "routineNames size != routineDescs size!\n";
    }

    void AutonSelector::control(int i)
    {
        // Change cursor position for middle button (pressed when i = 0)
        if (!i)
            incrementCursor(2);
        else
            changeMatchInfo(i);
        view();
    }

    void AutonSelector::view() const
    {
        pros::lcd::clear();
        std::string colorText{matchInfo.color == Color::Red ? "RED" : "BLUE"};
        std::string routineText{routineNames[matchInfo.routine]};
        if (!cursor)
            colorText += " <<<";
        else
            routineText += " <<<";
        pros::lcd::set_text(0, colorText);
        pros::lcd::set_text(1, routineText);
        printDesc(2, routineDescs[matchInfo.routine]);
    }

    MatchInfo AutonSelector::getMatchInfo() const
    {
        return matchInfo;
    }

    void AutonSelector::changeMatchInfo(int i)
    {
        // If cursor is over the color section (cursor == 1), change color, otherwise change routine
        if (!cursor)
            matchInfo.color = matchInfo.color == Color::Red ? Color::Blue : Color::Red;
        else
        {
            int routineNumber = matchInfo.routine + i + routineNames.size();
            matchInfo.routine = (Routine)(routineNumber % routineNames.size());
        }
    }

    void AutonSelector::setColor(const Color &color)
    {
        matchInfo.color = color;
    }
    
    Color AutonSelector::getColor() const
    {
        return matchInfo.color;
    }
}