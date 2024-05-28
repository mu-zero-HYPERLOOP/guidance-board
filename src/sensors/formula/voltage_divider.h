#pragma once

#include "util/metrics.h"
namespace sensors::formula {


// refer to for namings https://en.wikipedia.org/wiki/Voltage_divider
static Resistance r1_of_voltage_divider(Voltage v_in, Voltage v_out, Resistance r2) {
  return (v_in * r2 / v_out) - r2;
}

static Resistance r2_of_voltage_divider(Voltage v_in, Voltage v_out, Resistance r1) {
  return (v_out * r1) / (v_in - v_out);
}

static Voltage vout_of_voltage_divider(Voltage v_in, Resistance r1, Resistance r2) {
  return (r2 * v_in) / (r1 + r2);
}

static Voltage vin_of_voltage_divider(Voltage v_out, Resistance r1, Resistance r2) {
  return (v_out * (r1 + r2)) / r2;
}

}
