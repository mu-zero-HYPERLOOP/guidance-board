#include "sensors/magnet_temperatures.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/ntc_beta.h"
#include "sensors/formula/voltage_divider.h"
#include "util/boxcar.h"
#include "util/metrics.h"
#include <cassert>

static DMAMEM BoxcarFilter<Temperature, 10> l1_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> l2_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> r1_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> r2_filter(24_Celcius);


static Temperature sensor_helper(const Voltage& v) {
  return 0_Celcius;
}


static void on_l1_value(const Voltage& v) {
  const Temperature temp = sensor_helper(v);
  l1_filter.push(temp);
}

static void on_l2_value(const Voltage& v) {
  const Temperature temp = sensor_helper(v);
  l2_filter.push(temp);
}

static void on_r1_value(const Voltage& v) {
  const Temperature temp = sensor_helper(v);
  r1_filter.push(temp);
}

static void on_r2_value(const Voltage& v) {
  const Temperature temp = sensor_helper(v);
  r2_filter.push(temp);
}

void sensors::magnet_temperatures::begin() {
  assert(guidance_board::register_periodic_reading(MEAS_FREQUENCY,
      ain_pin::temp_sense_l1_20,
      on_l1_value));
  assert(guidance_board::register_periodic_reading(MEAS_FREQUENCY,
      ain_pin::temp_sense_l2_21,
      on_l2_value));
  assert(guidance_board::register_periodic_reading(MEAS_FREQUENCY,
      ain_pin::temp_sense_r1_14,
      on_r1_value));
  assert(guidance_board::register_periodic_reading(MEAS_FREQUENCY,
      ain_pin::temp_sense_r2_15,
      on_r2_value));
}

void sensors::magnet_temperatures::calibrate() {
  for (size_t i = 0; i < l1_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_l1_20);
    on_l1_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < l2_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_l2_21);
    on_l2_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < r1_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_r1_14);
    on_r1_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < r2_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::temp_sense_r2_15);
    on_r2_value(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::magnet_temperatures::update() {
  // TODO error checks
}
