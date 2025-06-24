#pragma once

#include "util/metrics.h"
namespace sensors::formula {

static inline Voltage isolated_voltage(Voltage v) {
  return v * (static_cast<float>(52500.0f / (1500.0f * 1.5f * 0.4f)));
}

static inline Voltage inv_isolated_voltage(Voltage v) {
  return v / (static_cast<float>(52500.0f / (1500.0f * 1.5f * 0.4f)));
}

} // namespace sensors::formula
