#pragma once

#include <memory>
#include "atum8/misc/constants.hpp"
#include "pros/rtos.hpp"

namespace atum8
{
    class Controller
    {
    public:
        virtual double getOutput(double state, double reference) = 0;

        virtual double getOutput(double error) = 0;

        virtual double getOutput() const;

        virtual void reset();

    protected:
        bool sampleTimePassed();
        double output{0};

    private:
        long unsigned int prevTime{0};
    };

    using UPController = std::unique_ptr<Controller>;
    using SPController = std::shared_ptr<Controller>;
}