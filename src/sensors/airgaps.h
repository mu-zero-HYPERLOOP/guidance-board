#pragma once

#include "util/metrics.h"
namespace sensors::airgaps {

constexpr Frequency MEAS_FREQUENCY = 1_kHz;

constexpr Resistance R_MEAS = 120_Ohm;

void begin();

void calibrate();

Distance conv_left_mag(Voltage v);
Distance conv_right_mag(Voltage v);
Distance conv_left_lim(Voltage v);
Distance conv_right_lim(Voltage v);

void update();

}
