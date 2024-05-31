#pragma once

#include "util/metrics.h"
namespace sensors::vdc {

constexpr Frequency MEAS_FREQUENCY = 1_kHz;


void begin();

void calibrate();

void update();

}
