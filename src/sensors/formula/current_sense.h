#pragma once

#include "util/metrics.h"
namespace sensors::formula {

constexpr Current OFFSET = 1.8_A;

constexpr Current current_sense(Voltage v, float gain, Resistance shunt_value) {
   return ((v / (1.5 * 0.4) - 2.5_V) / gain) / shunt_value + OFFSET;
}

constexpr Voltage inv_current_sense(Current i, float gain) {
  return (Voltage((static_cast<float>(i) * gain) * 1e-3) + 2.5_V) * 1.5 * 0.4;
}


 
}
