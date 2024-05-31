#include "control.h"
#include "canzero/canzero.h"
#include "util/metrics.h"
#include "util/trapazoidal_integral.h"
#include <algorithm>
#include <avr/pgmspace.h>
#include "util/ema.h"

volatile error_flag control_error;

/* static DMAMEM CsvWriter<6> csv_writer; */

void control::begin() {

  /* std::array<const char *, csv_writer.columns()> csv_headers = { */
  /*     "timestamp", "voltage", "target_current", "current", "i_term", "p_term", */
  /* }; */
  /* csv_writer.print_header(csv_headers); */
}

static __attribute__((always_inline)) Current
displacment_pid(Distance magnet_airgap_left, Distance lim_airgap_left,
                Distance magnet_airgap_right, Distance lim_airgap_rifht) {
  return 0_A;
}

static TrapazoidalIntegral current_pid_integral(0.0);
static ExponentialMovingAverage<float> d_filter(0.1);

static Timestamp SOR = Timestamp::now();
GuidancePwmControl FASTRUN control::control_loop(Current current_left,
                                                 Current current_right,
                                                 Distance magnet_airgap_left,
                                                 Distance lim_airgap_left,
                                                 Distance magnet_airgap_right,
                                                 Distance lim_airgap_right) {

  const float input = canzero_get_gamepad_lsb_x();

  const Current target_current = 5_A + 10_A * (input + 1) / 2.0;

  /* constexpr float R = 1.0; */
  /* constexpr float L = 0.07; */
  constexpr float Kp = 1.3f;
  constexpr float Ki = 0.75f;

  const float error_now = static_cast<float>(target_current - current_left);
  d_filter.push(error_now);
  const float error = d_filter.get();

  current_pid_integral.integrate(error, 1.0 / pwm::frequency());
  current_pid_integral.clamp(-100, 100);
  const float integral = current_pid_integral.get();

  float out = integral * Ki + error * Kp;

  const Voltage v = Voltage(std::clamp(out, -25.0f, 25.0f));

  /* if (canzero_get_gamepad_x_down()) { */
  /*   const Timestamp now = Timestamp::now(); */
  /*   const std::array<float, csv_writer.columns()> row = { */
  /*       static_cast<float>((now - SOR).as_us() * 1e-6), */
  /*       static_cast<float>(v), */
  /*       static_cast<float>(target_current), */
  /*       static_cast<float>(current_left), */
  /*       integral * Ki, */
  /*       error * Kp}; */
  /*   csv_writer.push_from_isr(row); */
  /* } */

  // Disable PWM ouput, can't be overwritten
  // by the state machine, if not in idle.
  /* pwm_brake::brake_immeditaly(); */

  // Shutdown complete 45V system immeditaly
  // can't be overwriten by the state machine,
  // if not in idle.
  /* sdc_brake::brake_immediatly(); */

  // TODO PID controller

  float dutyLeft = v / 45_V;
  float dutyRight = 0.0f;

  GuidancePwmControl pwmControl;
  pwmControl.left_l = 0.5 + dutyLeft / 2;
  pwmControl.left_r = 0.5 - dutyLeft / 2;
  pwmControl.right_l = 0.5 + dutyRight / 2;
  pwmControl.right_r = 0.5 - dutyRight / 2;
  return pwmControl;
}

void FASTRUN control::update() { 
  /* csv_writer.consume();  */
}
