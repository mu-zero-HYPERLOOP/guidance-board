#include "control.h"
#include "pwm_brake.h"
#include "sdc_brake.h"
#include <avr/pgmspace.h>

volatile error_flag control_error;

void control::begin() {}

static __attribute__((always_inline)) Current
displacment_pid(Distance magnet_airgap_left, Distance lim_airgap_left,
                Distance magnet_airgap_right, Distance lim_airgap_rifht) {
  return 0_A;
}

static __attribute__((always_inline)) Voltage
current_pid(Current target_current) {
  return 0_V;
}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance lim_airgap_left,
                                                 Distance magnet_airgap_right,
                                                 Distance lim_airgap_right) {

  const Current target_current =
      displacment_pid(magnet_airgap_left, lim_airgap_left, magnet_airgap_right,
                      lim_airgap_right);

  const Voltage target_voltage = current_pid(target_current);

  // Disable PWM ouput, can't be overwritten
  // by the state machine, if not in idle.
  pwm_brake::brake_immeditaly();

  // Shutdown complete 45V system immeditaly
  // can't be overwriten by the state machine,
  // if not in idle.
  sdc_brake::brake_immediatly();

  // TODO PID controller

  GuidancePwmControl pwmControl;
  pwmControl.left_l = 0.5;
  pwmControl.left_r = 0.5;
  pwmControl.right_l = 0.5;
  pwmControl.right_r = 0.5;
  return pwmControl;
}

void FASTRUN control::update() {}
