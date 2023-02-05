#include "slider.hpp"

namespace atum8
{
    Slider::Slider(UPController iCloseController,
                   UPController iFarController,
                   double iShiftLocation,
                   double iShiftSpeed) : closeController{std::move(iCloseController)},
                                         farController{std::move(iFarController)},
                                         shiftLocation{iShiftLocation},
                                         shiftSpeed{iShiftSpeed}
    {
    }

    double Slider::getOutput(double state, double reference)
    {
        return getOutput(reference - state);
    }

    double Slider::getOutput(double error)
    {
        const double outputClose{closeController->getOutput(error)};
        const double outputFar{farController->getOutput(error)};
        const double weightClose{weight(error)};
        const double weightFar{1 - weightClose};
        output = weightClose * outputClose + weightFar * outputFar;
        return output;
    }

    double Slider::weight(double error) const
    {
        // Complicated function to simulate a sigmoid, graph to get a better feel.
        const double numerator{tanh(-shiftSpeed * (abs(error) - shiftLocation))};
        return (numerator + 1) / 2;
    }
}