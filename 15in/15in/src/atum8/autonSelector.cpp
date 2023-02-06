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
            changeCursorPosition();
        else
            changeMatchInfo(i);
        view();
    }

    void AutonSelector::view() const
    {
        pros::lcd::clear();
        std::string colorText{matchInfo.color == Color::Red ? "RED" : "BLUE"};
        std::string routineText{routineNames[matchInfo.routine]};
        if (cursorPosition == CursorPosition::OverColor)
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

    void AutonSelector::printDesc(int line, const std::string &concatLines) const
    {
        std::string currentLine{""};
        int lineOffset{0};
        for (char c : concatLines)
        {
            currentLine += c;
            if (currentLine.size() == brainScreenWidth || c == '\n')
            {
                pros::lcd::set_text(line + lineOffset, currentLine);
                lineOffset++;
                currentLine = "";
            }
        }
        pros::lcd::set_text(line + lineOffset, currentLine);
    }

    void AutonSelector::changeCursorPosition()
    {
        if (cursorPosition == CursorPosition::OverColor)
            cursorPosition = CursorPosition::OverRoutine;
        else
            cursorPosition = CursorPosition::OverColor;
    }

    void AutonSelector::changeMatchInfo(int i)
    {
        if (cursorPosition == CursorPosition::OverColor)
            matchInfo.color = matchInfo.color == Color::Red ? Color::Blue : Color::Red;
        else
        {
            int routineNumber = matchInfo.routine + i + routineNames.size();
            matchInfo.routine = (Routine)(routineNumber % routineNames.size());
        }
    }
}