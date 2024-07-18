#include "control.h"
#include "canzero/canzero.h"
#include "sdc_brake.h"
#include "util/metrics.h"
#include <algorithm>
#include <avr/pgmspace.h>

void control::begin() {
}


GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance lim_airgap_left,
                                                 Distance magnet_airgap_right,
                                                 Distance lim_airgap_right) {


  // ====================== CURRENT PIDs =========================
  // Left current PI
  {
    const Current target = Current(left_current_pi_target);
    debug_left_target_current = target;
    pi_left_current_filter.push(current_left);
    const Current filtered_current = pi_left_current_filter.get();
    const float error = static_cast<float>(target - filtered_current);

    pi_left_current_state.p_term = error * pi_left_current_parameters.Kp;

    pi_left_current_state.i_term += error * pi_left_current_parameters.Ki;
    pi_left_current_state.i_term = std::clamp(pi_left_current_state.i_term,
                                              pi_left_current_parameters.i_min,
                                              pi_left_current_parameters.i_max);
  }
  float left_current_pi_output =
      pi_left_current_state.p_term + pi_left_current_state.i_term;

  // Right current PI
  {
    const Current target = Current(right_current_pi_target);
    debug_right_target_current = target;
    pi_right_current_filter.push(current_right);
    const Current filtered_current = pi_right_current_filter.get();
    const float error = static_cast<float>(target - filtered_current);

    pi_right_current_state.p_term = error * pi_right_current_parameters.Kp;

    pi_right_current_state.i_term += error * pi_right_current_parameters.Ki;
    pi_right_current_state.i_term = std::clamp(
        pi_right_current_state.i_term, pi_right_current_parameters.i_min,
        pi_right_current_parameters.i_max);
  }
  float right_current_pi_output =
      pi_right_current_state.p_term + pi_right_current_state.i_term;

  // =============== OUTPUT ==============

  constexpr float CONTROL_LOWER_BOUND = -0.9f;
  constexpr float CONTROL_UPPER_BOUND = 0.9f;

  const Voltage vdc = Voltage(canzero_get_vdc_voltage());

  // clamping here (just for debugging!) remove be on release!
  left_current_pi_output = std::clamp(
      left_current_pi_output, CONTROL_LOWER_BOUND * static_cast<float>(vdc),
      CONTROL_UPPER_BOUND * static_cast<float>(vdc));
  right_current_pi_output = std::clamp(
      right_current_pi_output, CONTROL_LOWER_BOUND * static_cast<float>(vdc),
      CONTROL_UPPER_BOUND * static_cast<float>(vdc));

  const Voltage voltage_left_magnet = Voltage(left_current_pi_output);
  const Voltage voltage_right_magnet = Voltage(right_current_pi_output);

  debug_left_voltage = voltage_left_magnet;
  debug_right_voltage = voltage_right_magnet;

  float controlLeft = voltage_left_magnet / vdc;
  float controlRight = voltage_right_magnet / vdc;

  controlLeft = std::clamp(controlLeft, -0.9f, 0.9f);
  controlRight = std::clamp(controlRight, -0.9f, 0.9f);

  float dutyLL = 0.5 + controlLeft / 2;
  float dutyLR = 0.5 - controlLeft / 2;
  float dutyRL = 0.5 + controlRight / 2;
  float dutyRR = 0.5 - controlRight / 2;

  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  return pwmControl;
}

void FASTRUN control::update() { 
}
