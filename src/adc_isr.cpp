#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "error_level_range_check.h"
#include "firmware/adc_etc.h"
#include "firmware/guidance_board.h"
#include "sensors/airgaps.h"
#include "sensors/magnet_current.h"
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
}

void adc_etc_done0_isr(AdcTrigRes res) {
  
  const Current i_mag_l = sensors::magnet_current::conv_left(res.trig_res(TRIG0, 0));
  const Current i_mag_r = sensors::magnet_current::conv_right(res.trig_res(TRIG0, 1));
  const Distance disp_sense_lim_l = sensors::airgaps::conv_left_lim(res.trig_res(TRIG0, 2));
  const Distance disp_sense_lim_r = sensors::airgaps::conv_right_lim(res.trig_res(TRIG4, 0));
  const Distance disp_sense_mag_r = sensors::airgaps::conv_right_mag(res.trig_res(TRIG4, 1));
  const Distance disp_sense_mag_l = sensors::airgaps::conv_left_mag(res.trig_res(TRIG4, 2));

  const GuidancePwmControl pwmControl = control::control_loop(
      i_mag_l, i_mag_r, disp_sense_mag_l, disp_sense_lim_l, disp_sense_mag_r,
      disp_sense_lim_r);
  pwm::control(static_cast<PwmControl>(pwmControl));
}

static Interval offline_interval(1_kHz);

void adc_isr::update() {
}
