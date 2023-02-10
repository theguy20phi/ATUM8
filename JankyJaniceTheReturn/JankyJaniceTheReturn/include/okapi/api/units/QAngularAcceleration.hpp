/*
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/units/RQuantity.hpp"

namespace okapi {
QUANTITY_TYPE(0, 0, -2, 1, QAngularAcceleration)

constexpr QAngularAcceleration rpmpm = (360 * degree) / (minute * minute);

inline namespace literals {
constexpr QAngularAcceleration operator"" _rpmpm(long double x) {
  return x * rpmpm;
}
constexpr QAngularAcceleration operator"" _rpmpm(unsigned long long int x) {
  return static_cast<double>(x) * rpmpm;
}
} // namespace literals
}
