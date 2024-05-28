#pragma once
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "firmware/adc_etc.h"
#include "sensors/formula/displacement420.h"

static constexpr float CURRENT_MEAS_GAIN_LEFT = 20;
static constexpr float CURRENT_MEAS_GAIN_RIGHT = 20;

static volatile Current i_mag_r;
static volatile Current i_mag_l;

static volatile Distance disp_sense_lim_l;
static volatile Distance disp_sense_lim_r;
static volatile Distance disp_sense_mag_l;
static volatile Distance disp_sense_mag_r;

void adc_etc_done0_isr(AdcTrigRes res) {

  const Voltage v_disp_sense_lim_l = res.trig_res(TRIG0, 0);
  const Voltage v_disp_sense_lim_r = res.trig_res(TRIG0, 1);
  const Voltage v_i_mag_r = res.trig_res(TRIG0, 2);

  const Voltage v_disp_sense_mag_l = res.trig_res(TRIG4, 0);
  const Voltage v_disp_sense_mag_r = res.trig_res(TRIG4, 1);
  const Voltage v_i_mag_l = res.trig_res(TRIG4, 2);

  // Current sensors.

  // TODO fixme nobody understands what's going on here
  i_mag_r = Current(-(static_cast<float>(v_i_mag_r) / 1.5 / 0.4 - 2.5) /
                    CURRENT_MEAS_GAIN_RIGHT / 1e-3);

  i_mag_l = Current(-(static_cast<float>(v_i_mag_l) / 1.5 / 0.4 - 2.5) /
                    CURRENT_MEAS_GAIN_LEFT / 1e-3);

  // Displacement sensors
  const Current i_disp_sense_lim_l = v_disp_sense_lim_l / 120_Ohm;
  disp_sense_lim_l = sensors::formula::displacement420(i_disp_sense_lim_l);

  const Current i_disp_sense_lim_r = v_disp_sense_lim_r / 120_Ohm;
  disp_sense_lim_r = sensors::formula::displacement420(i_disp_sense_lim_r);

  const Current i_disp_sense_mag_l = v_disp_sense_mag_l / 120_Ohm;
  disp_sense_mag_l = sensors::formula::displacement420(i_disp_sense_mag_l);

  const Current i_disp_sense_mag_r = v_disp_sense_mag_r / 120_Ohm;
  disp_sense_mag_r = sensors::formula::displacement420(i_disp_sense_mag_r);

  // perform all convertions

  const GuidancePwmControl pwmControl = control::control_loop(
      i_mag_l, i_mag_r, disp_sense_mag_l, disp_sense_lim_l, disp_sense_mag_r,
      disp_sense_lim_r);
  pwm::control(pwmControl);
}

void adc_isr::update() {

  canzero_set_current_left(static_cast<float>(i_mag_l));
  canzero_set_current_right(static_cast<float>(i_mag_r));

  canzero_set_outer_airgap_left(static_cast<float>(disp_sense_mag_l / 1_mm));
  canzero_set_inner_airgap_left(static_cast<float>(disp_sense_lim_l / 1_mm));
  canzero_set_outer_airgap_right(static_cast<float>(disp_sense_mag_r / 1_mm));
  canzero_set_inner_airgap_right(static_cast<float>(disp_sense_lim_r / 1_mm));
}
