#include "atum8/slewRate.hpp"
#include "main.h"

namespace atum8 {

double SlewRate::getOutput(float current, float desired,
                           float accelerationStepUp) {
  double step;

  if (fabs(current) < fabs(desired))
    step = accelerationStepUp;
  else
    step = accelerationStepDown;

  if (desired > current + step)
    return current + step;
  else if (desired < current - step)
    return current - step;
  else
    return desired;
}
} // namespace atum8
