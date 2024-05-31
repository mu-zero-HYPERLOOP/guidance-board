#include "sensors/magnet_temperatures.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "error_level_range_check.h"
#include "firmware/guidance_board.h"
#include "util/boxcar.h"
#include "util/metrics.h"
#include <algorithm>
#include <cassert>

static DMAMEM BoxcarFilter<Temperature, 10> l1_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> l2_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> r1_filter(24_Celcius);
static DMAMEM BoxcarFilter<Temperature, 10> r2_filter(24_Celcius);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_left(canzero_get_magnet_temperature_left_max,
                     canzero_get_error_level_config_magnet_temperature,
                     canzero_set_error_level_magnet_temperature_left);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_right(canzero_get_magnet_temperature_right_max,
                      canzero_get_error_level_config_magnet_temperature,
                      canzero_set_error_level_magnet_temperature_right);

static Temperature sensor_helper(const Voltage &v) { return 0_Celcius; }

static void on_l1_value(const Voltage &v) {
  const Temperature temp = sensor_helper(v);
  l1_filter.push(temp);
  canzero_set_magnet_temperature_left1(
      static_cast<float>(l1_filter.get() - 0_Celcius));
}

static void on_l2_value(const Voltage &v) {
  const Temperature temp = sensor_helper(v);
  l2_filter.push(temp);
  canzero_set_magnet_temperature_left2(
      static_cast<float>(l2_filter.get() - 0_Celcius));
}

static void on_r1_value(const Voltage &v) {
  const Temperature temp = sensor_helper(v);
  r1_filter.push(temp);
  canzero_set_magnet_temperature_right1(
      static_cast<float>(r1_filter.get() - 0_Celcius));
}

static void on_r2_value(const Voltage &v) {
  const Temperature temp = sensor_helper(v);
  r2_filter.push(temp);
  canzero_set_magnet_temperature_right2(
      static_cast<float>(r2_filter.get() - 0_Celcius));
}

void sensors::magnet_temperatures::begin() {
  canzero_set_magnet_temperature_left1(0);
  canzero_set_magnet_temperature_left2(0);
  canzero_set_magnet_temperature_left_max(0);
  canzero_set_magnet_temperature_right1(0);
  canzero_set_magnet_temperature_right2(0);
  canzero_set_magnet_temperature_right_max(0);
  canzero_set_error_level_magnet_temperature_left(error_level_OK);
  canzero_set_error_level_magnet_temperature_right(error_level_OK);
  canzero_set_error_level_config_magnet_temperature(error_level_config{
      .m_info_thresh = 45,
      .m_info_timeout = 5,
      .m_warning_thresh = 65,
      .m_warning_timeout = 5,
      .m_error_thresh = 80,
      .m_error_timeout = 5,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });

  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_l1_20, on_l1_value));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_l2_21, on_l2_value));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_r1_14, on_r1_value));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::temp_sense_r2_15, on_r2_value));
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
  float l1 = canzero_get_magnet_temperature_left1();
  float l2 = canzero_get_magnet_temperature_left2();
  float r1 = canzero_get_magnet_temperature_right1();
  float r2 = canzero_get_magnet_temperature_right2();

  float lmax = std::max(l1, l2);
  float rmax = std::max(r1, r2);
  canzero_set_magnet_temperature_left_max(lmax);
  canzero_set_magnet_temperature_right_max(rmax);

  error_check_left.check();
  error_check_right.check();
}
