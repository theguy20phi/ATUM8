#pragma once

#include "pros/llemu.hpp"
#include "atum8/misc/constants.hpp"
#include <string>
#include <memory>

namespace atum8
{
    class Gui
    {
    public:
        virtual void control(int i) = 0;

        virtual void view() const = 0;

    protected:
        void incrementCursor(int loop);
        void printDesc(int line, const std::string &concatLines) const;
        int cursor{0};
    };

    using UPGui = std::unique_ptr<Gui>;
    using SPGui = std::shared_ptr<Gui>;
}