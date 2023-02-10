#include "gui.hpp"

namespace atum8
{
    void Gui::incrementCursor(int loop)
    {
        cursor++;
        cursor %= loop;
    }

    void Gui::printDesc(int line, const std::string &concatLines) const
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
}