#include "pidFF.hpp"

namespace atum8
{
    PidFF::PidFF(double kP,
                 double kI,
                 double kD,
                 double FF,
                 double min,
                 double max) : PidFF::PidFF{PidFF::Parameters{kP, kI, kD, FF, min, max}} {}

    PidFF::PidFF(const PidFF::Parameters &iParams) : params{iParams} {}

    double PidFF::getOutput(double state, double reference)
    {
        if (!sampleTimePassed())
            return output;
        // Prevents derivative kick by using state diff rather than error diff
        const double D{params.kD * (state - prevState)};
        prevState = state;
        const double PI{getPI(reference - state)};
        const double FF{params.FF * reference};
        output = std::clamp(PI + D, params.min, params.max) + FF;
        return output;
    }

    double PidFF::getOutput(double error)
    {
        if (!sampleTimePassed())
            return output;
        const double D{params.kD * (error - prevError)};
        const double PI{getPI(error)};
        output = std::clamp(PI + D, params.min, params.max) + params.FF;
        return output;
    }

    double PidFF::getPI(double error)
    {
        const double P{params.kP * error};
        updateI(error);
        return P + I;
    }

    void PidFF::updateI(double error)
    {
        I += params.kI * error;
        // If we have crossed over the reference, I should be 0
        if (std::signbit(error) != std::signbit(prevError))
            I = 0;
        prevError = error;
        // Slowly forget earlier values
        I *= IDecay;
    }

    void PidFF::setKP(double kP)
    {
        params.kP = kP;
    }

    void PidFF::setKI(double kI)
    {
        params.kI = kI;
    }

    void PidFF::setKD(double kD)
    {
        params.kD = kD;
    }

    void PidFF::setFF(double FF)
    {
        params.FF = FF;
    }

    PidFF::Parameters PidFF::getParams() const
    {
        return params;
    }

    void PidFF::reset()
    {
        Controller::reset();
        prevError = 0;
        I = 0;
    }
}