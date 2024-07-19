#include "control.h"
#include "canzero/canzero.h"
#include "sdc_brake.h"
#include "util/boxcar.h"
#include "util/ema.h"
#include "util/metrics.h"
#include <algorithm>
#include <avr/pgmspace.h>

static ExponentialMovingAverage<Current> pi_left_current_filter{0.5, 0_A};
static ExponentialMovingAverage<Current> pi_right_current_filter{0.5, 0_A};
static BoxcarFilter<float, 50> pid_error_filter{0.0f};

struct PiState {
  float p_term;
  float i_term;
};

struct PiParameters {
  float Ki;
  float Kp;
  float i_max;
  float i_min;
};

struct PidState {
  float p_term;
  float i_term;
  float d_term;
  float last_error;
};
struct PidParameters {
  float Ki;
  float Kd;
  float Kp;
  float i_max;
  float i_min;
  float o_max;
};

static PiState pi_left_current_state;
static PiState pi_right_current_state;

static PiParameters pi_left_current_parameters;
static PiParameters pi_right_current_parameters;

static PidState pid_force_state;

static PidParameters pid_force_parameters;

void control::begin() {

  // Parameters
  pid_force_parameters.Kp = 70;
  pid_force_parameters.Ki = 0;
  pid_force_parameters.Kd = 0.6;

  pi_left_current_parameters.Kp = 20;
  pi_left_current_parameters.Ki = 10;
  pi_left_current_parameters.i_max = 30;
  pi_left_current_parameters.i_min = 0;


  pi_right_current_parameters.Ki = pi_left_current_parameters.Ki;
  pi_right_current_parameters.Kp = pi_left_current_parameters.Kp;
  pi_right_current_parameters.i_max = pi_left_current_parameters.i_max;
  pi_right_current_parameters.i_min = pi_left_current_parameters.i_min;

  pi_left_current_state.p_term = 0;
  pi_left_current_state.i_term = 0;

  pi_right_current_state.p_term = 0;
  pi_right_current_state.i_term = 0;

  pid_force_state.d_term = 0;
  pid_force_state.i_term = 0;
  pid_force_state.p_term = 0;
  pid_force_state.last_error = 0;
}

GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance lim_airgap_left,
                                                 Distance magnet_airgap_right,
                                                 Distance lim_airgap_right) {

  const Distance offset = (lim_airgap_left - lim_airgap_right) / 2.0f;
  const Distance target_offset = 0_m;

  // ================== FORCE PID CONTROLLER ====================
  const float error_raw = static_cast<float>(target_offset - offset);
  pid_error_filter.push(error_raw);
  const float error = pid_error_filter.get();

  pid_force_state.last_error = error;

  pid_force_state.p_term = error * pid_force_parameters.Kp;
  pid_force_state.i_term += error * pid_force_parameters.Ki;
  pid_force_state.d_term = (error - pid_force_state.last_error) *
                           static_cast<float>(pwm::frequency()) *
                           pid_force_parameters.Kd;

  const float force_target = pid_force_state.p_term + pid_force_state.i_term + pid_force_state.d_term;
  // ================ FORCE CURRENT CONVERTION ==================
  float current_target = 0;
  if (force_target >= 0) {
    current_target = std::sqrt(force_target);
  } else {
    current_target = -std::sqrt(-force_target);
  }

  current_target = std::clamp(current_target, -40.0f, 40.0f);

  float left_current_pi_target = 0;
  float right_current_pi_target = 0;
  if (current_target > 0) {
    left_current_pi_target = current_target;
  } else {
    right_current_pi_target = -current_target;
  }
  // ====================== CURRENT PIDs =========================
  // Left current PI
  {
    const Current target = Current(left_current_pi_target);
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

void FASTRUN control::update() {}
