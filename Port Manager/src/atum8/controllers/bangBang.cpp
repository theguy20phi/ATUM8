#include "bangBang.hpp"

namespace atum8
{
    BangBang::BangBang(ErrorOutputPairs iErrorOutputPairs, double iMaxOutput) : errorOutputPairs{std::move(iErrorOutputPairs)},
                                                                                maxOutput{iMaxOutput}
    {
        // Makes sure pairs are descendingly sorted by error so getOutput works
        std::sort(errorOutputPairs.begin(),
                  errorOutputPairs.end(),
                  std::greater<ErrorOutputPair>());
    }

    double BangBang::getOutput(double state, double reference)
    {
        return getOutput(reference - state);
    }

    double BangBang::getOutput(double error)
    {
        output = maxOutput;
        for (ErrorOutputPair errorOutputPair : errorOutputPairs)
        {
            // Assuming symmetry with error output pairs
            if (abs(error) <= errorOutputPair.first)
                output = errorOutputPair.second;
        }
        // Need to adjust if the error is negative
        if (std::signbit(error))
            output *= -1;
        return output;
    }
}