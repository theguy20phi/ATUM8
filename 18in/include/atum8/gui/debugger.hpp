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

        Debugger(const LineFns &iLineFns);

        void control(int i);

        void view() const;

    private:
        const LineFns lineFns;
    };

    using UPDebugger = std::unique_ptr<Debugger>;
    using SPDebugger = std::shared_ptr<Debugger>;
}