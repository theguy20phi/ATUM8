#pragma once

#include "controller.hpp"
#include <cmath>

namespace atum8
{
    class PidFF : public Controller
    {
    public:
        struct Parameters
        {
            double kP{0};
            double kI{0};
            double kD{0};
            double FF{0};
            double min{-12000};
            double max{120000};
        };

        PidFF(double kP = 0,
              double kI = 0,
              double kD = 0,
              double FF = 0,
              double min = -12000,
              double max = 12000);

        PidFF(const Parameters &iParams);

        double getOutput(double state, double reference);

        double getOutput(double error);

        void setKP(double kP);

        void setKI(double kI);

        void setKD(double kD);

        void setFF(double FF);

        Parameters getParams() const;

        void reset();

    private:
        static constexpr double IDecay{0.99};

        double getPI(double state);
        void updateI(double error);
        Parameters params;
        double prevError{0};
        double prevState{0};
        double I{0};
    };

    using UPPidFF = std::unique_ptr<PidFF>;
    using SPPidFF = std::shared_ptr<PidFF>;
}