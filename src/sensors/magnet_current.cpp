#include "sensors/magnet_current.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "error_level_range_check.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/current_sense.h"
#include "util/boxcar.h"
#include <cassert>

static DMAMEM BoxcarFilter<Current, 500> left_filter(0_A);
static DMAMEM BoxcarFilter<Current, 500> right_filter(0_A);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    left_magnet_check(canzero_get_current_left,
                canzero_get_error_level_config_magnet_current,
                canzero_set_error_level_magnet_current_left);
static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    right_magnet_check(canzero_get_current_right,
                canzero_get_error_level_config_magnet_current,
                canzero_set_error_level_magnet_current_right);

static void on_left_current(const Voltage &v) {
  const Current i = sensors::magnet_current::conv_left(v);
  left_filter.push(i);
  canzero_set_current_left(static_cast<float>(i));
}

static void on_right_current(const Voltage &v) {
  const Current i = sensors::magnet_current::conv_right(v);
  right_filter.push(i);
  canzero_set_current_right(static_cast<float>(i));
}

Current sensors::magnet_current::conv_left(Voltage v) {
  return sensors::formula::current_sense(v, sensors::magnet_current::SENSE_GAIN,
                                         sensors::magnet_current::SHUNT_R);
}
Current sensors::magnet_current::conv_right(Voltage v) {
  return sensors::formula::current_sense(v, sensors::magnet_current::SENSE_GAIN,
                                         sensors::magnet_current::SHUNT_R);
}

void sensors::magnet_current::begin() {
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::i_mag_l_24, on_left_current));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::i_mag_r_25, on_right_current));
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_config_magnet_current(error_level_config{
      .m_info_thresh = 30,
      .m_info_timeout = 0.5,
      .m_warning_thresh = 40,
      .m_warning_timeout = 0.5,
      .m_error_thresh = 40,
      .m_error_timeout = 2,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
}

void sensors::magnet_current::calibrate() {
  for (size_t i = 0; i < left_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::i_mag_l_24);
    on_left_current(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < left_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::i_mag_r_25);
    on_right_current(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

static Timestamp ready_current_ok_left = Timestamp::now();
static Timestamp ready_current_ok_right = Timestamp::now();

void sensors::magnet_current::update() {
  if (canzero_get_state() == guidance_state_READY) {
    const auto now = Timestamp::now();
    if (canzero_get_current_left() < 10){
      ready_current_ok_left = now;
    }
    if (now - ready_current_ok_left > 1_s){
      canzero_set_error_magnet_current_left_unexpected(error_flag_ERROR);
    }
    if (canzero_get_current_right() < 10){
      ready_current_ok_right = now;
    }
    if (now - ready_current_ok_right > 1_s){
      canzero_set_error_magnet_current_right_unexpected(error_flag_ERROR);
    }
  }else {
    ready_current_ok_right = Timestamp::now();
    ready_current_ok_left = Timestamp::now();
  }
  left_magnet_check.check();
  right_magnet_check.check();
}
