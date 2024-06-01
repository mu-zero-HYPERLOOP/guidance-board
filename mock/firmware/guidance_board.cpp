#include "firmware/guidance_board.h"
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "firmware/ain_scheduler.h"
#include "firmware/xbar.h"
#include "sensors/formula/current_sense.h"
#include "sensors/formula/displacement420.h"
#include "sensors/formula/isolated_voltage.h"
#include "sensors/input_current.h"
#include "util/boxcar.h"
#include "util/interval.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <thread>

constexpr size_t MAX_AIN_PERIODIC_JOBS = 10;

static AinScheduler<MAX_AIN_PERIODIC_JOBS> ain_scheduler;
static std::random_device rd{};
static std::mt19937 gen{rd()};

static Interval isr_interval(1_min);

void FLASHMEM guidance_board::begin() {
  xbar::begin();
  isr_interval = Interval(pwm::frequency());
}

static Voltage mock_disp(Distance distance) {
  std::normal_distribution dist{static_cast<float>(distance), 10e-6f};
  const Distance d = Distance(dist(gen));
  const Current i = sensors::formula::inv_displacement420(d);
  const Voltage v = i * adc_isr::DISP_MEAS_R;
  return v;
}

static Voltage mock_current(Current current, float gain) {
  std::normal_distribution dist{static_cast<float>(current), 1.0f};
  const Current i = Current(dist(gen));
  const Voltage v = sensors::formula::inv_current_sense(i, gain);
  return v;
}

extern PwmControl __pwm_control;

BoxcarFilter<Voltage, 1000> vdc_voltage(0_V);

static float current_left = 0;

Voltage FASTRUN guidance_board::sync_read(ain_pin pin) {
  switch (pin) {
  case ain_pin::disp_sense_mag_l_19:
    return mock_disp(40_mm);
  case ain_pin::disp_sense_lim_l_18:
    return mock_disp(40_mm);
  case ain_pin::disp_sense_mag_r_17:
    return mock_disp(40_mm);
  case ain_pin::disp_sense_lim_r_16:
    return mock_disp(40_mm);
  case ain_pin::temp_sense_l2_21:
  case ain_pin::temp_sense_l1_20:
  case ain_pin::temp_sense_r2_15:
  case ain_pin::temp_sense_r1_14:
    return 0_V;
  case ain_pin::vdc_sense_40: {
    switch (canzero_get_state()) {
    case guidance_state_INIT:
    case guidance_state_IDLE:
    case guidance_state_ARMING45:
      vdc_voltage.push(0_V);
      break;
    case guidance_state_PRECHARGE:
    case guidance_state_READY:
    case guidance_state_CONTROL:
      vdc_voltage.push(45_V);
      break;
    }
    std::normal_distribution dist{static_cast<float>(vdc_voltage.get()), 0.1f};
    const Voltage v_dist = Voltage(dist(gen));
    return sensors::formula::inv_isolated_voltage(v_dist);
  }
  case ain_pin::i_mag_l_24: {
    return mock_current(Current(current_left), adc_isr::CURRENT_MEAS_GAIN_RIGHT);
  }
  case ain_pin::i_mag_r_25: {
    const float duty = (__pwm_control.duty42 - 0.5) * 2.0;
    return mock_current(Current(duty * 38), adc_isr::CURRENT_MEAS_GAIN_RIGHT);
  }
  case ain_pin::i_mag_total: {
    const float duty1 = (__pwm_control.duty13 - 0.5) * 2.0;
    const float duty2 = (__pwm_control.duty42 - 0.5) * 2.0;
    return mock_current(Current((duty1 + duty2) * 38),
                        sensors::input_current::INPUT_CURRENT_GAIN);
  }
  default:
    assert(false && "trying to read a invalid pin");
    return 0_V;
  }
}

static std::normal_distribution mcu_temperature{273.15 + 24.0, 1.0};

Temperature guidance_board::read_mcu_temperature() {
  return Temperature(mcu_temperature(gen));
}

bool FLASHMEM guidance_board::register_periodic_reading(
    const Time &period, ain_pin pin, void (*on_value)(const Voltage &v)) {
  return ain_scheduler.schedule(period, AinSchedulerJob{
                                            .pin = pin,
                                            .on_value = on_value,
                                        });
}

/* void FASTRUN guidance_board::set_digital(ctrl_pin pin, bool state) { */
/*   // pass */
/* } */

void guidance_board::delay(Duration delta) {
  std::this_thread::sleep_for(
      std::chrono::duration<uint32_t, std::micro>(delta.as_us()));
}

guidance_board::InterruptLock FASTRUN guidance_board::InterruptLock::acquire() {
  return InterruptLock();
}

void FASTRUN guidance_board::InterruptLock::release() { m_acquired = false; }
FASTRUN guidance_board::InterruptLock::~InterruptLock() { m_acquired = false; }

extern void adc_etc_done0_isr(AdcTrigRes);
extern void adc_etc_done1_isr(AdcTrigRes);

using namespace std::chrono;
static high_resolution_clock::time_point last = high_resolution_clock::now();

void guidance_board::update() {
  ain_scheduler.update_continue();

  if (isr_interval.next()) {
    AdcTrigRes res;
    if (pwm::trig0_is_enabled()) {
      adc_etc_done0_isr(res);
    }
    if (pwm::trig1_is_enabled()) {
    }
  }

  const high_resolution_clock::time_point now = high_resolution_clock::now();
  const high_resolution_clock::duration dt_duration = now - last;
  float dt = duration_cast<duration<float, std::ratio<1>>>(dt_duration).count();
  last = now;

  
  constexpr float L = 0.07;
  constexpr float R = 1;
  
  const float duty = (__pwm_control.duty42 - 0.5) * 2.0;
  const float v_t = duty * 45.0;

  // Using implicit (backward) Euler method
  current_left = (current_left + dt * v_t / L) / (1 + dt * R / L);
}
