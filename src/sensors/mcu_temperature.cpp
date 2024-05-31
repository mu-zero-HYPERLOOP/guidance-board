#include "sensors/mcu_temperature.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "error_level_range_check.h"
#include "firmware/guidance_board.h"
#include "util/boxcar.h"
#include "util/interval.h"
#include "util/metrics.h"

static DMAMEM BoxcarFilter<Temperature, 10> filter(24_Celcius);

static Interval interval(sensors::mcu_temperature::MEAS_FREQUENCY);

static DMAMEM ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check(canzero_get_mcu_temperature,
                canzero_get_error_level_config_mcu_temperature,
                canzero_set_error_level_mcu_temperature);

void sensors::mcu_temperature::begin() {
  canzero_set_mcu_temperature(0);
  canzero_set_error_level_mcu_temperature(error_level_OK);
  canzero_set_error_level_config_mcu_temperature(error_level_config{
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
}

void sensors::mcu_temperature::calibrate() {
  for (size_t i = 0; i < filter.size(); ++i) {
    filter.push(guidance_board::read_mcu_temperature());
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::mcu_temperature::update() {
  if (interval.next()) {
    const Temperature temp = guidance_board::read_mcu_temperature();
    filter.push(temp);
    canzero_set_mcu_temperature(static_cast<float>(filter.get() - 0_Celcius));
  }
  error_check.check();
}
