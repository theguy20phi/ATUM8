#pragma once

#include "controller.hpp"
#include <cmath>

namespace atum8
{
    class Slider : public Controller
    {
    public:
        Slider(UPController iCloseController,
               UPController iFarController,
               double iShiftLocation,
               double iShiftSpeed);

        double getOutput(double state, double reference);

        double getOutput(double error);

    private:
        double weight(double error) const;
        UPController closeController;
        UPController farController;
        double shiftLocation{0};
        double shiftSpeed{0};
    };

    using UPSlider = std::unique_ptr<Slider>;
    using SPSlider = std::shared_ptr<Slider>;
}