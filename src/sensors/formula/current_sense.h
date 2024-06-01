#pragma once

#include "util/metrics.h"
namespace sensors::formula {

constexpr Current current_sense(Voltage v, float gain) {
  return Current(static_cast<float>(((v / (1.5 * 0.4) - 2.5_V) / gain) * 1e3));
}

constexpr Voltage inv_current_sense(Current i, float gain) {
  return Voltage(static_cast<float>(i));
}


 
}
