#pragma once

#include "controller.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace atum8
{
    class BangBang : public Controller
    {
    public:
        using ErrorOutputPair = std::pair<double, double>;
        using ErrorOutputPairs = std::vector<ErrorOutputPair>;

        BangBang(ErrorOutputPairs iErrorOutputPairs, double iMaxOutput);

        double getOutput(double state, double reference);

        double getOutput(double error);

    private:
        const double maxOutput;
        ErrorOutputPairs errorOutputPairs;
    };

    using UPBangBang = std::unique_ptr<BangBang>;
    using SPBangBang = std::shared_ptr<BangBang>;
}