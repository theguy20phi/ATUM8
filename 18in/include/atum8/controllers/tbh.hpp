#pragma once

#include "controller.hpp"
#include <cmath>

namespace atum8
{
    class Tbh : public Controller
    {
    public:
        Tbh(double iKTbh, double iTbhInit = 0.0);

        double getOutput(double state, double reference);

        double getOutput(double error);

        void setKTbh(double iKTbh);

        double getKTbh() const;

        void reset();

    private:
        double feedForward{0};
        double kTbh{0};
        double tbhInit{0};
        double prevError{0};
    };

    using UPTbh = std::unique_ptr<Tbh>;
    using SPTbh = std::shared_ptr<Tbh>;
}