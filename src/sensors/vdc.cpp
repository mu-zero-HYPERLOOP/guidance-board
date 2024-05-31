#include "sensors/vdc.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include <cassert>
#include "error_level_range_check.h"
#include "sensors/formula/isolated_voltage.h"
#include "util/boxcar.h"

static DMAMEM BoxcarFilter<Voltage, 10> filter(0_V);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check(canzero_get_vdc_voltage,
                     canzero_get_error_level_config_vdc_voltage,
                     canzero_set_error_level_vdc_voltage);

static void on_value(const Voltage &v) {
  // TODO convert voltage
  filter.push(sensors::formula::isolated_voltage(v));
  canzero_set_vdc_voltage(static_cast<float>(filter.get()));
}


void sensors::vdc::begin() {
  canzero_set_vdc_voltage(0);
  canzero_set_error_level_vdc_voltage(error_level_OK);
  canzero_set_error_level_config_vdc_voltage(error_level_config{
      .m_info_thresh = 45,
      .m_info_timeout = 5,
      .m_warning_thresh = 48,
      .m_warning_timeout = 1,
      .m_error_thresh = 50,
      .m_error_timeout = 1,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::vdc_sense_40, on_value));
}

void sensors::vdc::calibrate() {
  for (size_t i = 0; i < filter.size(); ++i) {
    on_value(guidance_board::sync_read(ain_pin::vdc_sense_40));
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::vdc::update() {
  error_check.check();
}
