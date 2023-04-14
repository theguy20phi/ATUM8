#include "atum8/algorithms/bangBang.hpp"
#include "main.h"


namespace atum8 {
    BangBang::BangBang(bool hasThreshold_) {
        hasThreshold = hasThreshold_;
    }
    BangBang::BangBang(bool hasThreshold_, double threshold_) {
        hasThreshold = hasThreshold_;
        threshold = threshold_;
    }

    bool BangBang::getOutput(double current, double desired)
    {
        return true;
    }
}