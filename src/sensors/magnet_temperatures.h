#pragma once

#include "util/metrics.h"
namespace sensors::magnet_temperatures {


constexpr Frequency MEAS_FREQUENCY = 100_Hz;

constexpr Resistance R_MEAS = 1_kOhm;
constexpr float NTC_BETA = 3900;
constexpr Resistance NTC_R_REF = 10_kOhm;
constexpr Temperature NTC_T_REF = 25_Celcius;

void begin();

void calibrate();

void update();

} // namespace sensors::magnet_temperatures
