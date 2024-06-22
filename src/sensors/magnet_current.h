#pragma once

#include "util/metrics.h"
namespace sensors::magnet_current {

constexpr Frequency MEAS_FREQUENCY = 918_Hz;

constexpr float SENSE_GAIN = 20.0f;
constexpr Resistance SHUNT_R = 1_mOhm;

void begin();

void calibrate();

Current conv_left(Voltage v);
Current conv_right(Voltage v);

void update();

}
