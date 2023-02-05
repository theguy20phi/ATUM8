#include "tbh.hpp"

namespace atum8
{

    Tbh::Tbh(double iKTbh) : kTbh{iKTbh} {}

    double Tbh::getOutput(double state, double reference)
    {
        return getOutput(reference - state);
    }

    double Tbh::getOutput(double error)
    {
        if(!sampleTimePassed())
            return output;
        output += kTbh * error;
        if (std::signbit(error) == std::signbit(prevError))
            return output;
        output = 0.5 * (output + feedForward);
        feedForward = output;
        prevError = error;
        return output;
    }

    void Tbh::setKTbh(double iKTbh)
    {
        kTbh = iKTbh;
    }

    double Tbh::getKTbh() const
    {
        return kTbh;
    }
}