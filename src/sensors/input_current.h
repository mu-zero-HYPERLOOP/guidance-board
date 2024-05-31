#pragma once

#include "util/metrics.h"

namespace sensors::input_current {

constexpr Frequency MEAS_FREQUENCY = 1_kHz;
constexpr float INPUT_CURRENT_GAIN = 20;

void begin();

void calibrate();

void update();

};
