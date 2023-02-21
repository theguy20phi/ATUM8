#include "pidFF.hpp"

namespace atum8
{
    PidFF::PidFF(double kP, double kI, double kD, double FF) : PidFF::PidFF{PidFF::Parameters{kP, kI, kD, FF}} {}

    PidFF::PidFF(const PidFF::Parameters &iParams) : params{iParams} {}

    // Quite a bit of duplication here, should later be fixed
    double PidFF::getOutput(double state, double reference)
    {
        if (!sampleTimePassed())
            return output;
        const double error{reference - state};
        const double P{params.kP * error};
        updateI(error);
        const double errorDiff{error - prevError};
        prevError = error;
        // Prevents derivative kick when reference changes
        const double D{(abs(errorDiff) > 1 / params.kD) ? 0 : params.kD * errorDiff};
        const double FF{params.FF * reference};
        output = P + I + D + FF;
        return output;
    }

    // Quite a bit of duplication here, should later be fixed
    double PidFF::getOutput(double error)
    {
        if (!sampleTimePassed())
            return output;
        const double P{params.kP * error};
        updateI(error);
        const double errorDiff{error - prevError};
        prevError = error;
        // Prevents derivative kick when reference changes
        const double D{(abs(errorDiff) > 1 / params.kD) ? 0 : params.kD * errorDiff};
        output = P + I + D + params.FF;
        return output;
    }

    void PidFF::updateI(double error)
    {
        I += params.kI * error;
        // If we have crossed over the reference, I should be 0
        if (std::signbit(error) != std::signbit(prevError))
            I = 0;
        // Generally decent clamping for I value
        I = std::clamp(I, -1 / params.kI, 1 / params.kI);
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