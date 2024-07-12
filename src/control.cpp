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

  const Voltage v_left = 0_V;
  const Voltage v_right = 0_V;

  const Voltage vdc = Voltage(canzero_get_vdc_voltage());

  const float controlLeft = std::clamp(v_left / vdc, -0.9f, 0.9f);
  const float controlRight = std::clamp(v_right / vdc, -0.9f, 0.9f);

  float dutyLL = 0.5 + controlLeft / 2;
  float dutyLR = 0.5 - controlLeft / 2;
  float dutyRL = 0.5 + controlRight / 2;
  float dutyRR = 0.5 - controlRight / 2;

  dutyLL = 0.0f;
  dutyLR = 0.0f;
  dutyRL = 0.0f;
  dutyRR = 0.0f;

  dutyLL = std::clamp(dutyLL, 0.1f, 0.9f);
  dutyLR = std::clamp(dutyLR, 0.1f, 0.9f);
  dutyRL = std::clamp(dutyRL, 0.1f, 0.9f);
  dutyRR = std::clamp(dutyRR, 0.1f, 0.9f);


  GuidancePwmControl pwmControl{};
  pwmControl.left_l = dutyLL;
  pwmControl.left_r = dutyLR;
  pwmControl.right_l = dutyRL;
  pwmControl.right_r = dutyRR;
  return pwmControl;
}

void FASTRUN control::update() { 
}
