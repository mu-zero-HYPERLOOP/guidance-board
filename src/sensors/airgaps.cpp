#include "sensors/airgaps.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "sensors/formula/displacement420.h"
#include "util/boxcar.h"
#include <cassert>
#include "print.h"

static DMAMEM BoxcarFilter<Distance, 10> left_mag_filter(0_mm);
static DMAMEM BoxcarFilter<Distance, 10> right_mag_filter(0_mm);
static DMAMEM BoxcarFilter<Distance, 10> left_lim_filter(0_mm);
static DMAMEM BoxcarFilter<Distance, 10> right_lim_filter(0_mm);

static Distance offset_left_mag = 0_m;
static Distance offset_right_mag = 0_m;
static Distance offset_left_lim = 0_m;
static Distance offset_right_lim = 0_m;

Distance sensors::airgaps::conv_left_mag(Voltage v){
  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_left_mag;
}
Distance sensors::airgaps::conv_right_mag(Voltage v){
  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_right_mag;
}
Distance sensors::airgaps::conv_left_lim(Voltage v){
  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_left_lim;
}
Distance sensors::airgaps::conv_right_lim(Voltage v){
  const Current i = v / sensors::airgaps::R_MEAS;
  return sensors::formula::displacement420(i) + offset_right_lim;
}


static void on_left_mag_disp(const Voltage &v) {
  if (v < 0.1_V) {
    canzero_set_error_outer_airgap_left_invalid(error_flag_ERROR);
    canzero_set_outer_airgap_left(0);
    return;
  }
  canzero_set_error_outer_airgap_left_invalid(error_flag_OK);
  const Distance disp = sensors::airgaps::conv_left_mag(v);
  left_mag_filter.push(disp);
  canzero_set_outer_airgap_left(left_mag_filter.get() / 1_mm);
}

static void on_right_mag_disp(const Voltage &v) {
  if (v < 0.1_V) {
    canzero_set_error_outer_airgap_right_invalid(error_flag_ERROR);
    canzero_set_outer_airgap_right(0);
    return;
  }
  canzero_set_error_outer_airgap_right_invalid(error_flag_OK);
  const Distance disp = sensors::airgaps::conv_right_mag(v);
  right_mag_filter.push(disp);
  canzero_set_outer_airgap_right(right_mag_filter.get() / 1_mm);
}

static void on_left_lim_disp(const Voltage &v) {
  if (v < 0.1_V) {
    canzero_set_error_inner_airgap_left_invalid(error_flag_ERROR);
    canzero_set_inner_airgap_left(0);
    return;
  }
  canzero_set_error_inner_airgap_left_invalid(error_flag_OK);
  const Distance disp = sensors::airgaps::conv_left_lim(v);
  left_lim_filter.push(disp);
  canzero_set_inner_airgap_left(left_lim_filter.get() / 1_mm);
}

static void on_right_lim_disp(const Voltage &v) {
  if (v < 0.1_V) {
    canzero_set_error_inner_airgap_right_invalid(error_flag_ERROR);
    canzero_set_inner_airgap_right(0);
    return;
  }
  canzero_set_error_inner_airgap_right_invalid(error_flag_OK);
  const Distance disp = sensors::airgaps::conv_right_lim(v);
  right_lim_filter.push(disp);
  canzero_set_outer_airgap_right(right_lim_filter.get() / 1_mm);
}

void sensors::airgaps::begin() {
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_l_19, on_left_mag_disp));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_mag_r_17, on_right_mag_disp));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_lim_l_18, on_left_lim_disp));
  assert(guidance_board::register_periodic_reading(
      MEAS_FREQUENCY, ain_pin::disp_sense_lim_r_16, on_right_lim_disp));
}

void sensors::airgaps::calibrate() {

  // probably a hardcoded calibration required!
  offset_left_mag = 0_mm;
  offset_right_mag = 0_mm;
  offset_left_lim = 0_mm;
  offset_right_lim = 0_mm;

  for (size_t i = 0; i < left_mag_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_l_19);
    on_left_mag_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < right_mag_filter.size(); ++i) {
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_mag_r_17);
    on_right_mag_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for (size_t i = 0; i < left_lim_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_lim_l_18);
    on_left_lim_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  for(size_t i = 0; i < right_lim_filter.size(); ++i){
    const Voltage v = guidance_board::sync_read(ain_pin::disp_sense_lim_r_16);
    on_right_lim_disp(v);
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
  canzero_set_inner_airgap_left(left_lim_filter.get() / 1_mm);
  canzero_set_outer_airgap_left(left_mag_filter.get() / 1_mm);
  canzero_set_inner_airgap_right(right_lim_filter.get() / 1_mm);
  canzero_set_outer_airgap_right(right_mag_filter.get() / 1_mm);
}

void sensors::airgaps::update() {}
