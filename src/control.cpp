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


  float left_current_pi_output = 0.0f;
  float right_current_pi_output = 0.0f;
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
