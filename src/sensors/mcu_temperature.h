#pragma once 

#include "util/metrics.h"

namespace sensors::mcu_temperature {

constexpr Frequency MEAS_FREQUENCY = 100_Hz;

void begin();

void calibrate();

void update();

}
