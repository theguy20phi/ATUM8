#include "debugger.hpp"

namespace atum8
{
    Debugger::Debugger(const LineFns &iLineFns) : lineFns{iLineFns}
    {
    }

    void Debugger::control(int i)
    {
        if (!i)
            incrementCursor(lineFns.size());
        else
            lineFns[cursor](i);
        view();
    }

    void Debugger::view() const
    {
        pros::lcd::clear();
        for (int i{cursor}, line{0}; line < brainScreenHeight && i < lineFns.size(); i++, line++)
            pros::lcd::set_text(line, lineFns[i](0) + (!line ? " <<<" : ""));
    }
}