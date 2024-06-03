#pragma once

#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/pwm.h"
namespace pwm_brake {

extern volatile bool brake_engaged;
extern volatile bool running;

static void begin() { 
  pwm::disable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();
  brake_engaged = false; 
  running = false;

}

// Immediatly disables the PWM outputs and the ISR
// this action cannot be reset with request_start
// it requires release_brake to be called to enable the
// PWM again, which only happens when the
// board is in the idle state.
static __attribute__((always_inline)) void brake_immeditaly() {
  brake_engaged = true;
  running = false;
  pwm::disable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();
}

static __attribute__((always_inline)) void release_brake() {
  brake_engaged = false;
}

inline bool request_start() {
  if (brake_engaged) {
    pwm::disable_output();
    pwm::disable_trig0();
    pwm::disable_trig1();
    running = false;
    return false;
  } else {
    pwm::enable_output();
    pwm::enable_trig0();
    pwm::enable_trig1();
    running = true;
    return true;
  }
}

inline void stop() {
  pwm::disable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();
  running = false;
}

inline void update() {
  canzero_set_control_active(running ? bool_t_TRUE : bool_t_FALSE);
}
} // namespace pwm_brake
