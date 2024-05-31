#pragma once

#include "util/metrics.h"

namespace adc_isr {

static constexpr float CURRENT_MEAS_GAIN_LEFT = 20;
static constexpr float CURRENT_MEAS_GAIN_RIGHT = 20;

static constexpr Resistance DISP_MEAS_R = 120_Ohm;

void begin();

void update();
}
