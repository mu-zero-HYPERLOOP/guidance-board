#pragma once

#include "util/metrics.h"

namespace sensors::input_current {

constexpr Frequency MEAS_FREQUENCY = 1_kHz;
constexpr float SENSE_GAIN = 20;
constexpr Resistance INPUT_SHUNT_R = 1_mOhm;

void begin();

void calibrate();

void update();

};
