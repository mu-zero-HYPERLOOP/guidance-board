#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "error_level_range_check.h"
#include "firmware/adc_etc.h"
#include "firmware/guidance_board.h"
#include "pwm_brake.h"
#include "sensors/formula/current_sense.h"
#include "sensors/formula/displacement420.h"
#include "util/interval.h"
#include <avr/pgmspace.h>

using namespace adc_isr;

static volatile Current i_mag_r;
static volatile Current i_mag_l;

static volatile Distance disp_sense_lim_l;
static volatile Distance disp_sense_lim_r;
static volatile Distance disp_sense_mag_l;
static volatile Distance disp_sense_mag_r;

static ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_current_left(canzero_get_current_left,
                             canzero_get_error_level_config_magnet_current,
                             canzero_set_error_level_magnet_current_left);

static ErrorLevelRangeCheck<EXPECT_UNDER>
    error_check_current_right(canzero_get_current_right,
                              canzero_get_error_level_config_magnet_current,
                              canzero_set_error_level_magnet_current_right);

void adc_isr::begin() {
  canzero_set_gamepad_x_down(bool_t_FALSE);
  // NOTE: if we decise to use any filters initalize them here!
  canzero_set_current_left(0);
  canzero_set_current_right(0);
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_config_magnet_current(error_level_config{
      .m_info_thresh = 15,
      .m_info_timeout = 2,
      .m_warning_thresh = 20,
      .m_warning_timeout = 1,
      .m_error_thresh = 25,
      .m_error_timeout = 1,
      .m_ignore_info = bool_t_FALSE,
      .m_ignore_warning = bool_t_FALSE,
      .m_ignore_error = bool_t_FALSE,
  });
  {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_lim_l_18);
    const Current i = v / DISP_MEAS_R;
    disp_sense_lim_l = sensors::formula::displacement420(i);
    canzero_set_inner_airgap_left(static_cast<float>(disp_sense_lim_l) * 1e3);
  }
  {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_lim_r_16);
    const Current i = v / DISP_MEAS_R;
    disp_sense_lim_r = sensors::formula::displacement420(i);
    canzero_set_inner_airgap_right(static_cast<float>(disp_sense_lim_r) * 1e3);
  }
  {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_l_19);
    const Current i = v / DISP_MEAS_R;
    disp_sense_mag_l = sensors::formula::displacement420(i);
    canzero_set_outer_airgap_left(static_cast<float>(disp_sense_mag_l) * 1e3);
  }
  {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_r_17);
    const Current i = v / DISP_MEAS_R;
    disp_sense_mag_r = sensors::formula::displacement420(i);
    canzero_set_outer_airgap_right(static_cast<float>(disp_sense_mag_r) * 1e3);
  }
}

void adc_etc_done0_isr(AdcTrigRes res) {

  const Voltage v_i_mag_l_1 = res.trig_res(TRIG0, 0);
  const Voltage v_i_mag_l_2 = res.trig_res(TRIG0, 1);
  const Voltage v_i_mag_l_3 = res.trig_res(TRIG0, 2);
  const Voltage v_i_mag_l_4 = res.trig_res(TRIG0, 3);
  const Voltage v_i_mag_l_5 = res.trig_res(TRIG0, 4);
  const Voltage v_i_mag_l_6 = res.trig_res(TRIG0, 5);
  const Voltage v_i_mag_l = (v_i_mag_l_1 + v_i_mag_l_2 + v_i_mag_l_3 +
                             v_i_mag_l_4 + v_i_mag_l_5 + v_i_mag_l_6) /
                            6.0f;
  const Voltage v_disp_sense_lim_l = res.trig_res(TRIG0, 6);
  const Voltage v_disp_sense_lim_r = res.trig_res(TRIG0, 7);

  const Voltage v_i_mag_r_1 = res.trig_res(TRIG4, 0);
  const Voltage v_i_mag_r_2 = res.trig_res(TRIG4, 1);
  const Voltage v_i_mag_r_3 = res.trig_res(TRIG4, 2);
  const Voltage v_i_mag_r_4 = res.trig_res(TRIG4, 3);
  const Voltage v_i_mag_r_5 = res.trig_res(TRIG4, 4);
  const Voltage v_i_mag_r_6 = res.trig_res(TRIG4, 5);
  const Voltage v_i_mag_r = (v_i_mag_r_1 + v_i_mag_r_2 + v_i_mag_r_3 +
                             v_i_mag_r_4 + v_i_mag_r_5 + v_i_mag_r_6) /
                            6.0f;
  const Voltage v_disp_sense_mag_l = res.trig_res(TRIG4, 6);
  const Voltage v_disp_sense_mag_r = res.trig_res(TRIG4, 7);

  // Current sensors.

  // TODO fixme nobody understands what's going on here
  i_mag_r = sensors::formula::current_sense(v_i_mag_r, CURRENT_MEAS_GAIN_RIGHT);
  i_mag_l = sensors::formula::current_sense(v_i_mag_l, CURRENT_MEAS_GAIN_LEFT);

  // Displacement sensors
  const Current i_disp_sense_lim_l = v_disp_sense_lim_l / DISP_MEAS_R;
  disp_sense_lim_l = sensors::formula::displacement420(i_disp_sense_lim_l);

  const Current i_disp_sense_lim_r = v_disp_sense_lim_r / DISP_MEAS_R;
  disp_sense_lim_r = sensors::formula::displacement420(i_disp_sense_lim_r);

  const Current i_disp_sense_mag_l = v_disp_sense_mag_l / DISP_MEAS_R;
  disp_sense_mag_l = sensors::formula::displacement420(i_disp_sense_mag_l);

  const Current i_disp_sense_mag_r = v_disp_sense_mag_r / DISP_MEAS_R;
  disp_sense_mag_r = sensors::formula::displacement420(i_disp_sense_mag_r);

  const GuidancePwmControl pwmControl = control::control_loop(
      i_mag_l, i_mag_r, disp_sense_mag_l, disp_sense_lim_l, disp_sense_mag_r,
      disp_sense_lim_r);
  pwm::control(static_cast<PwmControl>(pwmControl));
}

static Interval offline_interval(1_kHz);

void adc_isr::update() {

  if (!pwm::trig0_is_enabled()) {
    if (offline_interval.next()) {
      const Voltage v_i_mag_l = guidance_board::sync_read(ain_pin::i_mag_l_24);
      const Voltage v_disp_sense_lim_l =
          guidance_board::sync_read(ain_pin::disp_sense_lim_l_18);
      const Voltage v_disp_sense_lim_r =
          guidance_board::sync_read(ain_pin::disp_sense_lim_r_16);

      const Voltage v_i_mag_r = guidance_board::sync_read(ain_pin::i_mag_r_25);
      const Voltage v_disp_sense_mag_l =
          guidance_board::sync_read(ain_pin::disp_sense_mag_l_19);
      const Voltage v_disp_sense_mag_r =
          guidance_board::sync_read(ain_pin::disp_sense_mag_r_17);

      // Current sensors.

      // TODO fixme nobody understands what's going on here
      i_mag_r =
          sensors::formula::current_sense(v_i_mag_r, CURRENT_MEAS_GAIN_RIGHT);
      i_mag_l =
          sensors::formula::current_sense(v_i_mag_l, CURRENT_MEAS_GAIN_LEFT);

      // Displacement sensors
      const Current i_disp_sense_lim_l = v_disp_sense_lim_l / DISP_MEAS_R;
      disp_sense_lim_l = sensors::formula::displacement420(i_disp_sense_lim_l);

      const Current i_disp_sense_lim_r = v_disp_sense_lim_r / DISP_MEAS_R;
      disp_sense_lim_r = sensors::formula::displacement420(i_disp_sense_lim_r);

      const Current i_disp_sense_mag_l = v_disp_sense_mag_l / DISP_MEAS_R;
      disp_sense_mag_l = sensors::formula::displacement420(i_disp_sense_mag_l);

      const Current i_disp_sense_mag_r = v_disp_sense_mag_r / DISP_MEAS_R;
      disp_sense_mag_r = sensors::formula::displacement420(i_disp_sense_mag_r);
    }
  } else {
    offline_interval.reset();
  }

  canzero_set_current_left(static_cast<float>(i_mag_l));
  canzero_set_current_right(static_cast<float>(i_mag_r));

  canzero_set_outer_airgap_left(static_cast<float>(disp_sense_mag_l) * 1e3);
  canzero_set_inner_airgap_left(static_cast<float>(disp_sense_lim_l) * 1e3);
  canzero_set_outer_airgap_right(static_cast<float>(disp_sense_mag_r) * 1e3);
  canzero_set_inner_airgap_right(static_cast<float>(disp_sense_lim_r) * 1e3);

  error_check_current_left.check();
  error_check_current_right.check();
}
