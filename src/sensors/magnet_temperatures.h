#pragma once

#include "util/metrics.h"
namespace sensors::magnet_temperatures {


constexpr Frequency MEAS_FREQUENCY = 100_Hz;

void begin();

void calibrate();

void update();

} // namespace sensors::magnet_temperatures
