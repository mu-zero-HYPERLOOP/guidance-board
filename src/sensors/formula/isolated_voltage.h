#pragma once

#include "util/metrics.h"
namespace sensors::formula{

static inline Voltage isolated_voltage(Voltage v) {
  return v * static_cast<float>(52500 / (1500 * 1.5 * 0.4));
}

static inline Voltage inv_isolated_voltage(Voltage v) {
  return v / static_cast<float>(52500 / (1500.0f * 1.5 * 0.4));
}


}
