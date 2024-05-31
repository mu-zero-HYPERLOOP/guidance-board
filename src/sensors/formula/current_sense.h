#pragma once

#include "util/metrics.h"
namespace sensors::formula {

constexpr Current current_sense(Voltage v, float gain) {
  return Current(static_cast<float>(v));
}

constexpr Voltage inv_current_sense(Current i, float gain) {
  return Voltage(static_cast<float>(i));
}


 
}
